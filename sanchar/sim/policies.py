"""SANCHAR policy for the engine: N independent Coordinators joined only by a lossy signed bus.

Every broadcast is wrapped in an Envelope, HMAC-signed, MessagePack-encoded, and delivered to each other
robot independently subject to packet loss, dead zones and partitions. Receivers enforce replay
protection. Bytes on air and per-robot decision time are measured for the edge-readiness slide.
"""
from __future__ import annotations

import random
import time
from dataclasses import replace

from sanchar.protocol.messages import (PROTOCOL_VERSION, Bye, Envelope, Hello, ObstacleDelta, ReplayGuard,
                                       Seen, Signer, decode, encode)
from sanchar.robot.coordinator import Config, Coordinator
from sanchar.sim.engine import World


class SancharPolicy:
    name = "sanchar"

    def __init__(self, cfg: Config | None = None, bye_repeats: int = 5, relay_hops: int = 2):
        self.cfg = cfg or Config()
        self.bye_repeats = bye_repeats
        self.relay_hops = relay_hops

    def reset(self, world: World, seed: int) -> None:
        self.rng = random.Random(seed * 104729 + 17)
        ids = sorted(world.pos)
        self.agents = {r: Coordinator(world.rm, r, replace(self.cfg, seed=seed), roster=set(ids)) for r in ids}
        for r, a in self.agents.items():
            a.set_pose(world.pos[r])
        self.signer = Signer()
        self.seq = {r: 0 for r in ids}
        self.guard = {r: ReplayGuard() for r in ids}
        self.leaving: dict[int, int] = {}          # rid -> remaining Bye repeats
        self.relay_q: dict[int, list[tuple[bytes, Envelope, int]]] = {r: [] for r in ids}
        self.relayed: dict[int, set[tuple[int, int]]] = {r: set() for r in ids}
        self.relay_packets = 0
        self.bytes_sent = 0
        self.packets_sent = 0
        self.packets_delivered = 0
        self.rejected = 0
        self.step_times: list[float] = []
        self.events: list[tuple[int, int, str]] = []
        self.last_decisions: dict = {}
        self.tap = getattr(self, "tap", None)

    def set_goal(self, rid, goal, task_id, load) -> None:
        self.agents[rid].set_goal(goal, task_id, load)

    def remove(self, rid, world) -> None:
        self.leaving[rid] = self.bye_repeats

    def _wrap(self, rid: int, body) -> tuple[bytes, Envelope]:
        self.seq[rid] += 1
        env = self.signer.sign(Envelope(ver=PROTOCOL_VERSION, sender=rid, seq=self.seq[rid],
                                        hlc=0, hops=0, body=body))
        data = encode(env)
        self.bytes_sent += len(data)
        self.packets_sent += 1
        return data, env

    def _broadcast(self, world: World, outbox, positions: dict[int, int], rnd: int) -> None:
        """One radio round: fresh broadcasts plus relays of Hellos heard last round (multi-hop)."""
        receivers = [(r, positions.get(r, world.pos[r])) for r in sorted(world.alive)]
        packets: list[tuple[int, int, bytes, Envelope, int]] = []   # (tx robot, tx node, data, env, hops_left)
        for src, src_node, bodies in outbox:
            for body in bodies:
                data, env = self._wrap(src, body)
                packets.append((src, src_node, data, env, self.relay_hops))
        for rel in sorted(self.relay_q):
            if rel not in world.alive:
                continue
            for data, env, hops in self.relay_q[rel]:
                packets.append((rel, positions.get(rel, world.pos[rel]), data, env, hops))
                self.bytes_sent += len(data)
                self.packets_sent += 1
                self.relay_packets += 1
            self.relay_q[rel] = []
        for tx, tx_node, data, _, hops in packets:
            env = decode(data)                      # what every receiver decodes
            if not self.signer.verify(env):
                self.rejected += 1
                continue
            if self.tap is not None and tx == env.sender:   # passive listener (dashboard), same radio
                self.tap(tx, tx_node, env.body, rnd, len(data))
            for dst, dst_node in receivers:
                if dst == tx or dst == env.sender or not world.can_talk(tx, tx_node, dst, dst_node):
                    continue
                if self.rng.random() < world.loss:
                    continue
                if not self.guard[dst].accept(env.sender, env.seq):
                    continue                        # duplicate (already heard directly) or replay
                self._deliver(dst, env.body, rnd)
                self.packets_delivered += 1
                key = (env.sender, env.seq)
                if hops > 0 and isinstance(env.body, Hello) and key not in self.relayed[dst]:
                    self.relayed[dst].add(key)
                    self.relay_q[dst].append((data, env, hops - 1))

    def _deliver(self, dst: int, body, rnd: int) -> None:
        self.agents[dst].ingest(body, rnd)

    def _pre_claim(self, rid: int, world: World) -> list:
        return []

    def step(self, world: World) -> dict[int, int]:
        t = world.tick
        alive = sorted(world.alive)
        # claim round
        outbox = []
        for rid in alive:
            a = self.agents[rid]
            sensed = world.sensed_obstacles(rid)
            fresh_obs = [n for n in sensed if n not in a.obstacles]
            if sensed:
                a.report_obstacle(sensed)
            extra = self._pre_claim(rid, world)
            sightings = world.sensed_robots(rid)
            if sightings:
                a.observe(sightings, 2 * t)
                extra = extra + [Seen(robot_id=rid, tick=2 * t, seen=sightings)]
            t0 = time.perf_counter()
            hello, intent, lease, _ = a.claim_phase(t)
            self.step_times.append(time.perf_counter() - t0)
            bodies = [hello, intent] + extra
            if fresh_obs:
                bodies.append(ObstacleDelta(robot_id=rid, tick=t, blocked=fresh_obs,
                                            expires_tick=t + self.cfg.obstacle_ttl))
            outbox.append((rid, world.pos[rid], bodies))
        for rid, left in list(self.leaving.items()):
            node = world.pos[rid]
            outbox.append((rid, node, [Bye(robot_id=rid, tick=t, node=node)]))
            if left <= 1:
                del self.leaving[rid]
            else:
                self.leaving[rid] = left - 1
        self._broadcast(world, outbox, world.pos, 2 * t)
        # move round
        out: dict[int, int] = {}
        outbox = []
        for rid in alive:
            t0 = time.perf_counter()
            d = self.agents[rid].move_phase(t)
            self.step_times[-len(alive) + alive.index(rid)] += time.perf_counter() - t0
            self.last_decisions[rid] = d
            for e in d.events:
                self.events.append((t, rid, e))
            out[rid] = d.node
            outbox.append((rid, d.node, [d.hello]))
        self._broadcast(world, outbox, out, 2 * t + 1)
        return out

    def stats(self) -> dict:
        st = sorted(self.step_times) or [0.0]
        n_robot_ticks = max(1, len(self.step_times))
        return {
            "bytes_per_robot_tick": self.bytes_sent / n_robot_ticks,
            "packets_sent": self.packets_sent,
            "decision_ms_p50": 1000 * st[len(st) // 2],
            "decision_ms_p99": 1000 * st[min(len(st) - 1, int(len(st) * 0.99))],
            "rejected_packets": self.rejected,
            "relay_share": self.relay_packets / max(1, self.packets_sent),
            "pushes": sum(a.metrics["pushes"] for a in self.agents.values()),
            "cycles_broken": sum(a.metrics["cycles_broken"] for a in self.agents.values()),
        }
