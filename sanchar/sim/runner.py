"""Multi-process fleet: one OS process per robot, peer-to-peer over real UDP sockets.

    python -m sanchar.sim.runner --robots 6 --orders 30 [--loss 0.2] [--kill 2@40]

Processes:
  * robot i  : runs its own Coordinator + TaskMarket. It talks to peers ONLY by sending signed UDP
               datagrams to their ports (broadcast emulation on one machine; multicast on a LAN).
  * world    : the physical simulator and the WMS stub. It tells each robot what its own sensors
               would see, applies the moves robots choose, counts collisions, and injects orders.
               It never plans, never relays coordination messages, never assigns orders.
`--kill i@t` terminates robot i's process at tick t (like pulling its battery). The rest of the
fleet notices only because i goes silent, re-opens its orders and routes around its body.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import random
import socket
import time

from sanchar.protocol.messages import (PROTOCOL_VERSION, Bid, Envelope, Hello, ReplayGuard, Seen, Signer,
                                       TaskAnnounce, decode, encode)
from sanchar.robot.allocator import TaskMarket
from sanchar.robot.coordinator import Config, Coordinator
from sanchar.sim.engine import World, apply_moves
from sanchar.world.roadmap import WAREHOUSE_LAYOUT, Roadmap

HOST = "127.0.0.1"
WORLD_PORT = 26120
ROBOT_BASE = 26200


def _sock(port: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, port))
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    if hasattr(socket, "SIO_UDP_CONNRESET"):             # Windows: a dead peer must not break my socket
        s.ioctl(socket.SIO_UDP_CONNRESET, False)
    return s


# ------------------------------------------------------------------------------------------ robot
def robot_main(rid: int, n: int, seed: int, loss: float) -> None:
    rm = Roadmap.from_ascii(WAREHOUSE_LAYOUT)
    coord = Coordinator(rm, rid, Config(seed=seed), roster=set(range(n)) - {rid})
    market = TaskMarket(rm, rid, seed=seed)
    signer, guard = Signer(), ReplayGuard()
    rng = random.Random(seed * 131 + rid)
    sock = _sock(ROBOT_BASE + rid)
    peers = [(HOST, ROBOT_BASE + j) for j in range(n) if j != rid]
    seq = 0

    def send_peers(bodies, rnd):
        nonlocal seq
        for body in bodies:
            seq += 1
            data = b"P" + encode(signer.sign(Envelope(ver=PROTOCOL_VERSION, sender=rid, seq=seq, hlc=rnd, hops=0, body=body)))
            for addr in peers:
                sock.sendto(data, addr)

    def to_world(msg):
        sock.sendto(b"W" + json.dumps(msg).encode(), (HOST, WORLD_PORT))

    while True:
        try:
            data, _ = sock.recvfrom(65536)
        except (ConnectionResetError, OSError):
            continue
        if data[:1] == b"P":
            if rng.random() < loss:
                continue
            try:
                env = decode(data[1:])
            except Exception:
                continue
            if not signer.verify(env) or not guard.accept(env.sender, env.seq):
                continue
            if isinstance(env.body, (TaskAnnounce, Bid)):
                market.ingest(env.body, env.hlc // 2)
            else:
                coord.ingest(env.body, env.hlc)
            continue
        msg = json.loads(data[1:])
        if msg.get("stop"):
            left = [(tid, rec.status, rec.owner, rec.version) for tid, rec in sorted(market.tasks.items()) if rec.status != "done"]
            if left:
                print(f"R{rid} unfinished view: {left} my_task={market.my_task} node={rm.pos(coord.node)} goal={coord.goal}", flush=True)
            return
        r, t = msg["r"], msg["r"] // 2
        if msg.get("init") is not None:
            coord.set_pose(msg["init"])
        for o in msg.get("orders", []):
            market.inject(TaskAnnounce(task_id=o[0], pickup=o[1], dropoff=o[2], created_tick=t, origin=-1), t)
        coord.set_battery(msg["battery"])
        if msg["obstacles"]:
            coord.report_obstacle(msg["obstacles"])
        if r % 2 == 0:                                     # claim round
            alive = {p for p, pv in coord.peers.items() if r - pv.heard_round <= 6}
            extra = market.tick(t, coord, alive, msg["battery"], rm.chargers())
            if msg["sighted"]:
                seen = [tuple(x) for x in msg["sighted"]]
                coord.observe(seen, r)
                extra.append(Seen(robot_id=rid, tick=r, seen=seen))
            t0 = time.perf_counter()
            hello, intent, _, _ = coord.claim_phase(t)
            ms = 1000 * (time.perf_counter() - t0)
            send_peers([hello, intent] + extra, r)
            to_world({"rid": rid, "r": r, "ms": ms})
        else:                                              # move round
            d = coord.move_phase(t)
            send_peers([d.hello], r)
            done = [tid for tid, rec in market.tasks.items() if rec.status == "done"]
            to_world({"rid": rid, "r": r, "node": d.node, "reason": d.reason, "done": done})


# ------------------------------------------------------------------------------------------ world
def world_main(n: int, n_orders: int, seed: int, ticks: int, kill: dict[int, int], round_timeout: float,
               procs: dict[int, mp.Process]) -> dict:
    rm = Roadmap.from_ascii(WAREHOUSE_LAYOUT)
    rng = random.Random(seed)
    st = rm.stations()
    starts = rng.sample(st, n)
    world = World(rm, pos=dict(enumerate(starts)), alive=set(range(n)), present=set(range(n)))
    battery = {r: rng.uniform(0.6, 1.0) for r in range(n)}
    top = [s for s in st if rm.pos(s)[1] <= 4]
    bot = [s for s in st if rm.pos(s)[1] > 4]
    orders, t_next = [], 0.0
    for i in range(n_orders):
        t_next += rng.expovariate(1 / 3.0)
        a, b = (top, bot) if rng.random() < 0.5 else (bot, top)
        orders.append((int(t_next), f"T{i:03d}", rng.choice(a), rng.choice(b)))
    sock = _sock(WORLD_PORT)
    sock.settimeout(0.005)
    done_at, collisions, missed = {}, 0, {r: 0 for r in range(n)}
    removal: dict[int, int] = {}
    decision_ms, t0 = [], time.time()
    for r in range(2 * ticks):
        t = r // 2
        world.tick = t
        for rid, kt in list(kill.items()):
            if t >= kt and procs[rid].is_alive():
                procs[rid].terminate()                     # pull the battery
                print(f"[t={t}] killed robot process R{rid} (pid {procs[rid].pid})", flush=True)
                del kill[rid]
        due = [o for o in orders if o[0] == t] if r % 2 == 0 else []
        crashed = {world.pos[x] for x in world.present - world.alive}
        targets = set(rng.sample(sorted(world.alive), min(2, len(world.alive)))) if due else set()
        for rid in sorted(world.alive):
            me = world.pos[rid]
            msg = {"r": r, "battery": round(battery[rid], 3),
                   "obstacles": [m for m in rm.neighbors(me) if m in crashed],
                   "sighted": world.sensed_robots(rid) if r % 2 == 0 else []}
            if r == 0:
                msg["init"] = me
            if rid in targets:
                msg["orders"] = [[o[1], o[2], o[3]] for o in due]
            sock.sendto(b"W" + json.dumps(msg).encode(), (HOST, ROBOT_BASE + rid))
        replies, deadline = {}, time.time() + round_timeout
        while len(replies) < len(world.alive) and time.time() < deadline:
            try:
                data, _ = sock.recvfrom(65536)
            except (socket.timeout, ConnectionResetError, OSError):
                continue
            m = json.loads(data[1:])
            if m["r"] == r:
                replies[m["rid"]] = m
        time.sleep(0.004)                                  # let peer datagrams land before the next round
        for rid in sorted(world.alive):
            if rid in replies:
                missed[rid] = 0
            else:
                missed[rid] += 1
                if missed[rid] >= 6:                       # silent process: it is dead, its body stays
                    world.alive.discard(rid)
                    removal[rid] = t + 90
                    print(f"[t={t}] world: R{rid} stopped responding (body left at {rm.pos(world.pos[rid])})", flush=True)
        for rid, due_t in list(removal.items()):
            if t >= due_t:                             # technician wheels the dead robot away
                world.present.discard(rid)
                del removal[rid]
                print(f"[t={t}] world: technician removed R{rid}'s body", flush=True)
        if r % 2 == 0:
            decision_ms += [m["ms"] for m in replies.values()]
            continue
        prev = dict(world.pos)
        moves = {rid: m["node"] for rid, m in replies.items() if rid in world.alive}
        c, sw, oh, tp = apply_moves(world, moves)
        collisions += c + sw + oh
        for rid in world.alive:
            battery[rid] = max(0.0, battery[rid] - 0.0002 - (0.0015 if world.pos[rid] != prev[rid] else 0))
        for m in replies.values():
            for tid in m.get("done", []):
                done_at.setdefault(tid, t + 1)
        if t % 20 == 0:
            print(f"[t={t:4d}] alive {len(world.alive)}/{n}  orders done {len(done_at)}/{n_orders}  collisions {collisions}", flush=True)
        if len(done_at) == n_orders and t > orders[-1][0]:
            break
    for rid in range(n):
        sock.sendto(b"W" + json.dumps({"stop": True, "r": -1}).encode(), (HOST, ROBOT_BASE + rid))
    ms = sorted(decision_ms) or [0.0]
    return {"ticks": t + 1, "orders_done": len(done_at), "orders": n_orders, "collisions": collisions,
            "robots_alive": len(world.alive), "decision_ms_p50": ms[len(ms) // 2],
            "decision_ms_p99": ms[int(len(ms) * 0.99)], "wall_s": round(time.time() - t0, 1)}


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robots", type=int, default=6)
    ap.add_argument("--orders", type=int, default=30)
    ap.add_argument("--ticks", type=int, default=600)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--loss", type=float, default=0.0, help="drop this fraction of peer datagrams")
    ap.add_argument("--kill", action="append", default=[], help="R@T: kill robot R's process at tick T")
    ap.add_argument("--round-timeout", type=float, default=0.25)
    ap.add_argument("--out", default=None, help="write the result JSON here")
    a = ap.parse_args(argv)
    kill = {int(k.split("@")[0]): int(k.split("@")[1]) for k in a.kill}
    procs = {rid: mp.Process(target=robot_main, args=(rid, a.robots, a.seed, a.loss), daemon=True)
             for rid in range(a.robots)}
    for p in procs.values():
        p.start()
    time.sleep(1.0)
    print(f"started {a.robots} robot processes: " + ", ".join(f"R{r}=pid {p.pid}" for r, p in procs.items()), flush=True)
    res = world_main(a.robots, a.orders, a.seed, a.ticks, kill, a.round_timeout, procs)
    res.update({"robots": a.robots, "killed": sorted(int(k.split("@")[0]) for k in a.kill), "loss": a.loss, "seed": a.seed})
    print("RESULT", json.dumps(res), flush=True)
    if a.out:
        from pathlib import Path
        Path(a.out).write_text(json.dumps(res, indent=1))
    for p in procs.values():
        p.join(timeout=2)
        if p.is_alive():
            p.terminate()


if __name__ == "__main__":
    main()
