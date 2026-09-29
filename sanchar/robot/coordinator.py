"""Decentralized Guided-PIBT coordinator. One instance per robot; no shared memory, no master.

ROUND MODEL. Coordination runs faster than motion, as on real robots (10 Hz radio, 1 m/s drive).
Each motion tick t has two message rounds:
  * claim round  (round 2t):   claim_phase(t) picks the node to enter next and broadcasts
                               Hello(node, claim) + Intent (guide path).
  * move round   (round 2t+1): move_phase(t) enters the claimed node iff the SAFETY RULE holds
                               against the claims just heard, then broadcasts Hello(new node).
A claim on an OCCUPIED node is a PIBT push request ("please vacate"); it persists in the
move-round Hello until executed, so the occupant sees it next claim round.

SAFETY RULE (docs/DEADLOCK_FREEDOM.md), evaluated in move_phase(t):
  I enter m only if m was my claim in round 2t, and for every peer B I know:
    * FRESH (I hold B's claim-round Hello of round 2t):  B.node != m and B.claim != m;
    * STALE (last Hello in round r_B < 2t): m is outside reach(B), the set of nodes B can occupy
      after round 2t+1. B moves at most once per move round and only to a node it claimed, so
      reach is the ball of radius (#move rounds since r_B) around B.node, tightened to
      {B.node, B.claim} when the last thing I heard was B's claim.
  Plus: every roster member has been heard at least once (no invisible peers), and a robot that
  just booted holds still until peers have heard it.

PRIORITY: 5-tuple, larger = higher, identical on every node from broadcast state:
  (effective_load, task_age, -battery_bucket, -robot_id, 0)
  task_age = ticks since the current goal was assigned (PIBT's "elapsed"): it never resets on a
  move, so interacting robots keep a stable order. effective_load = 1 if carrying goods or
  starved (wait_age >= w_max). Contests compare my last BROADCAST key with peers' last broadcast
  keys, so both sides always agree on who wins.
"""
from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field

from sanchar.protocol.messages import Bye, Hello, Intent, Lease, ObstacleDelta, PriorityKey, Seen, Yield
from sanchar.robot.planner import ReservationTable, plan_guide_path, preference_order
from sanchar.world.roadmap import Roadmap


def claim_round(t: int) -> int:
    return 2 * t


def move_round(t: int) -> int:
    return 2 * t + 1


def _odd_upto(x: int) -> int:
    return (x + 1) // 2 if x >= 0 else 0


@dataclass
class Config:
    horizon: int = 10            # planning window (ticks)
    w_max: int = 8               # wait ticks before a robot escalates to the "loaded" class
    silence_stop_after: int = 3  # ticks without hearing anyone before I hold position (degraded)
    forget_after: int = 60       # ticks of silence before a peer is presumed removed
    boot_hold_ticks: int = 2     # a joining robot sits still this long so everyone hears it
    lease_ttl: int = 6
    obstacle_ttl: int = 40
    patience: int = 4            # ticks blocked on the same next node before a forced replan
    switch_margin: int = 3       # a new plan must be this many ticks shorter to replace a valid one
    lower_prio_reserve: int | None = None   # None: treat lower-priority peers as parked for the
                                            # horizon (detour); k: reserve k ticks then push through
    honour_planned_wait: bool = False       # True: a plan that waits makes the robot wait
    gate_stopped_blocks: bool = True        # a robot stopped inside a corridor blocks entry both ways
    gate_recheck_in_move: bool = False      # re-check the corridor gate against this round's claims
    yield_only_higher: bool = False         # hold yield mode only towards higher-priority robots
    livelock_after: int = 30                # ticks without getting closer to the goal before random escape
    escape_ticks: int = 6                   # length of a random-escape episode
    proactive_yield_within: int = 0         # yield if a higher-priority path reaches my node within k ticks (0=off)
    clear_mouth: bool = False               # a robot on a corridor mouth yields to robots exiting through it
    gate_mouths: bool = True                # wait one node before a corridor mouth while the lane is busy the other way
    exit_yield_within: int = 0              # yield to a robot leaving a corridor whose path reaches me within k ticks (0=off)
    ack_window: int = 2                     # ticks: I move only if every peer acked hearing me this recently
    exclude_after: int = 30                 # ticks of silence before a peer is dropped from my ack set
    seed: int = 0


@dataclass
class PeerView:
    hello: Hello                 # hello.tick holds the ROUND the message was sent in
    heard_round: int
    intent: list[int] | None = None
    intent_round: int = -1
    obs_node: int | None = None  # where someone physically saw it (onboard perception)
    obs_round: int = -1


@dataclass
class Decision:
    tick: int
    node: int
    moved: bool
    claim: int | None
    reason: str
    hello: Hello
    intent: Intent
    lease: Lease | None = None
    yield_: Yield | None = None
    events: list[str] = field(default_factory=list)


class Coordinator:
    def __init__(self, rm: Roadmap, robot_id: int, cfg: Config | None = None,
                 roster: set[int] | None = None):
        self.rm, self.id, self.cfg = rm, robot_id, cfg or Config()
        self.roster = set(roster or ()) - {robot_id}
        self.rng = random.Random(self.cfg.seed * 1000 + robot_id)
        self.node: int = -1
        self.prev_node: int | None = None
        self.goal: int | None = None
        self.task_id: str | None = None
        self.load = False
        self.battery = 1.0
        self.tick = 0
        self.boot_tick: int | None = None
        self.wait_age = 0
        self.age = 0
        self.claim: int | None = None
        self.moved_last = False
        self.bkey: PriorityKey | None = None
        self.guide: list[int] = []
        self.blocked_ticks = 0
        self.yield_to: dict[int, int] = {}
        self.best_dist = 10**9
        self.stuck = 0
        self.escape_left = 0
        self.peers: dict[int, PeerView] = {}
        self.obstacles: dict[int, int] = {}
        self.departed: set[int] = set()
        self.last_heard_any = -1          # round
        self.degraded = False
        self._pending: tuple[Intent, Lease | None, Yield | None, list[str], str] | None = None
        self.metrics = {"moves": 0, "waits": 0, "pushes": 0, "yields": 0, "replans": 0,
                        "cycles_broken": 0, "swaps_resolved": 0}

    # ---- external state ---------------------------------------------------------------------
    def set_pose(self, node: int) -> None:
        self.node, self.prev_node, self.claim = node, None, None

    def set_goal(self, goal: int | None, task_id: str | None = None, load: bool = False) -> None:
        self.goal, self.task_id, self.load = goal, task_id, load
        self.guide, self.wait_age, self.age = [], 0, 0
        self.best_dist, self.stuck, self.escape_left = 10**9, 0, 0

    def set_battery(self, b: float) -> None:
        self.battery = max(0.0, min(1.0, b))

    def report_obstacle(self, nodes: list[int]) -> None:
        for n in nodes:
            self.obstacles[n] = self.tick + self.cfg.obstacle_ttl

    # ---- ingest -----------------------------------------------------------------------------
    def ingest(self, body, local_round: int) -> None:
        """Transport has already verified the signature and replay seq. Duplicates are harmless."""
        if isinstance(body, Hello):
            if body.robot_id == self.id or body.robot_id in self.departed:
                return
            pv = self.peers.get(body.robot_id)
            if pv is None:
                self.peers[body.robot_id] = PeerView(body, local_round)
            elif body.tick >= pv.hello.tick:
                pv.hello, pv.heard_round = body, local_round
            self.last_heard_any = max(self.last_heard_any, local_round)
        elif isinstance(body, Intent):
            pv = self.peers.get(body.robot_id)
            if pv is not None and body.tick >= pv.intent_round:
                pv.intent, pv.intent_round = body.nodes, body.tick
        elif isinstance(body, ObstacleDelta):
            for n in body.blocked:
                self.obstacles[n] = max(self.obstacles.get(n, 0), body.expires_tick)
        elif isinstance(body, Seen):
            self.observe(body.seen, body.tick)
        elif isinstance(body, Bye):
            if body.robot_id != self.id:
                self.peers.pop(body.robot_id, None)
                self.roster.discard(body.robot_id)
                self.departed.add(body.robot_id)

    def observe(self, seen: list, rnd: int) -> None:
        """Physical sightings (mine or relayed via Seen): pin a silent robot's position."""
        for rid, node in seen:
            pv = self.peers.get(rid)
            if rid == self.id or pv is None:
                continue
            if rnd > max(pv.hello.tick, pv.obs_round):
                pv.obs_node, pv.obs_round = node, rnd

    # ---- priority ---------------------------------------------------------------------------
    def priority_key(self) -> PriorityKey:
        eff_load = 1 if (self.load or self.wait_age >= self.cfg.w_max) else 0
        return (eff_load, self.age, -int(self.battery * 4), -self.id, 0)

    def _prio(self, rid: int) -> PriorityKey:
        return tuple(self.peers[rid].hello.priority)

    # ---- views ------------------------------------------------------------------------------
    def _reach(self, pv: PeerView, r_eval: int) -> set[int]:
        """Nodes peer B can occupy after round r_eval, given the freshest thing I know about it.

        Ack rule: B may move in move round m only while it holds my acknowledgement that I heard
        it at a round >= m - 2*ack_window. My Hellos can never acknowledge more than the newest B
        Hello I hold (round h.tick), so B's last possible move is at round h.tick + 2*ack_window,
        UNLESS B drops me from its ack set, which it can do only while it is still hearing other
        robots, and no earlier than exclude_after ticks after it last heard me (t_free).
          * before t_free: ball around B's last known position (Hello or physical sighting) with
            radius = move rounds left in B's ack budget;
          * after t_free: B is either far away (out of short-range radio reach of everyone near
            me, so it cannot reach my next node without being heard) or dead. If someone sees it,
            its sighted node plus one hop; if nobody sees it, it is not a threat to my next move
            (a dead body next to me is caught by my own proximity sensing first)."""
        h = pv.hello
        t_free = h.heard.get(self.id, -1) + 2 * self.cfg.exclude_after
        observed = pv.obs_round > h.tick
        if r_eval > t_free:
            if observed and pv.obs_round >= r_eval - 4:
                return {pv.obs_node, *self.rm.neighbors(pv.obs_node)}
            return set()
        last_move = h.tick + 2 * self.cfg.ack_window + 1
        if observed:
            r0, base = pv.obs_round, {pv.obs_node}
        else:
            r0, base = h.tick, {h.node}
            if h.tick % 2 == 0 and h.claim is not None and h.claim != h.node:
                base = {h.node, h.claim}
                r0 = h.tick + 1                        # the next move round is spent on the claim
        k = _odd_upto(min(r_eval, last_move)) - _odd_upto(r0)
        if k <= 0:
            return base
        seen = set(base)
        q = deque((n, 0) for n in base)
        while q:
            n, d = q.popleft()
            if d >= k:
                continue
            for m in self.rm.neighbors(n):
                if m not in seen:
                    seen.add(m)
                    q.append((m, d + 1))
        return seen

    def _missing_acks(self, r: int) -> list[int]:
        """Roster peers that have not recently confirmed hearing me (I may not move until they do)."""
        missing = []
        isolated = r - self.last_heard_any > 2 * self.cfg.ack_window   # I hear nobody: I am the one cut off
        for rid in sorted(self.roster | set(self.peers)):
            if rid in self.departed:
                continue
            pv = self.peers.get(rid)
            if pv is None:
                missing.append(rid)
                continue
            if not isolated and r - pv.heard_round > 2 * self.cfg.exclude_after:
                continue                                   # presumed dead or far away: excluded
            if pv.hello.heard.get(self.id, -10**9) < r - 2 * self.cfg.ack_window:
                missing.append(rid)
        return missing

    def _fresh(self, pv: PeerView, r: int) -> bool:
        return pv.hello.tick >= r - 1

    def _views(self, r: int, r_eval: int) -> tuple[dict[int, int], set[int], dict[int, int]]:
        """occupant: node -> fresh peer; reach: nodes stale peers may hold; claims: node -> peer."""
        occupant, reach, claims = {}, set(), {}
        for rid, pv in self.peers.items():
            if self._fresh(pv, r):
                occupant[pv.hello.node] = rid
                if pv.hello.claim is not None and pv.hello.claim != pv.hello.node:
                    claims[pv.hello.claim] = rid
            else:
                reach |= self._reach(pv, r_eval)
        return occupant, reach, claims

    def _live_obstacles(self) -> set[int]:
        self.obstacles = {n: e for n, e in self.obstacles.items() if e > self.tick}
        return set(self.obstacles)

    # ---- conflict helpers -------------------------------------------------------------------
    def _peer_dir(self, rid: int, cid: int) -> int:
        pv = self.peers[rid]
        h = pv.hello
        path = pv.intent if pv.intent and pv.intent[0] == h.node else [h.node]
        if h.claim is not None and h.claim != h.node:
            path = [h.node, h.claim] + (path[2:] if len(path) > 1 and path[1] == h.claim else [])
        return self.rm.path_direction(cid, path)

    def _corridor_gate_blocked(self, m: int, r: int, key: PriorityKey, entering_only: bool = False) -> int | None:
        """Entering single-lane corridor node m from outside (an end or a pocket): wait if a fresh
        peer inside is heading the other way (or is stopped inside), or a higher-priority peer is
        entering it the other way right now. One traffic direction per corridor at a time."""
        cid = self.rm.corridor_of(m)
        if cid is None or self.rm.corridor_of(self.node) == cid:
            return None
        path = self.guide if len(self.guide) > 1 and self.guide[0] == self.node and self.guide[1] == m else [self.node, m]
        my_dir = self.rm.path_direction(cid, path)
        chain = self.rm.cidx[cid]
        for rid, pv in self.peers.items():
            if not self._fresh(pv, r):
                continue
            h = pv.hello
            inside = h.node in chain
            entering = h.claim in chain and not inside
            if not (inside or entering) or (entering_only and not entering):
                continue
            their = self._peer_dir(rid, cid)
            opposing = their != my_dir or (self.cfg.gate_stopped_blocks and (their == 0 or my_dir == 0))
            if opposing and (inside or self._prio(rid) > key):
                return rid
        return None

    def _mouth_gate_blocked(self, m: int, r: int, key: PriorityKey) -> int | None:
        """Stepping onto node m that is the mouth of a corridor my guide path enters next: treat it
        like entering the corridor, so I wait at a waiting point instead of in the doorway."""
        g = self.guide
        if m not in g[1:3]:
            return None
        i = g.index(m, 1)
        if i + 1 >= len(g):
            return None
        nxt = g[i + 1]
        cid = self.rm.corridor_of(nxt)
        if cid is None or self.rm.corridor_of(m) == cid or m not in self.rm.mouths[cid]:
            return None
        my_dir = self.rm.corridor_direction(cid, m, nxt)
        chain = self.rm.cidx[cid]
        for rid, pv in self.peers.items():
            if not self._fresh(pv, r):
                continue
            h = pv.hello
            if h.node in chain:
                their = self._peer_dir(rid, cid)
                if their != my_dir and their != 0:
                    return rid
        return None


    def _pushed_by(self, r: int, key: PriorityKey) -> int | None:
        """A fresh peer claiming my node with higher priority, or exiting a corridor I block."""
        best = None
        for rid, pv in self.peers.items():
            if not (self._fresh(pv, r) and pv.hello.claim == self.node):
                continue
            pc = self.rm.corridor_of(pv.hello.node)
            exiting = pc is not None and pc != self.rm.corridor_of(self.node)
            if self._prio(rid) > key or exiting:
                if best is None or self._prio(rid) > self._prio(best):
                    best = rid
        return best

    def _wants(self, rid: int) -> int | None:
        """Next node a peer wants: its pending claim, else the next node of its intent."""
        pv = self.peers[rid]
        h = pv.hello
        if h.claim is not None and h.claim != h.node:
            return h.claim
        if pv.intent and len(pv.intent) > 1 and pv.intent[0] == h.node and pv.intent[1] != h.node:
            return pv.intent[1]
        return None

    def _wait_cycle(self, r: int, occupant: dict[int, int]) -> list[int]:
        want = {rid: pv.hello.claim for rid, pv in self.peers.items() if self._fresh(pv, r)}
        want[self.id] = self.claim
        occ = dict(occupant)
        occ[self.node] = self.id
        path, cur = [], self.id
        while cur is not None and cur not in path:
            path.append(cur)
            target = want.get(cur)
            cur = occ.get(target) if target is not None else None
        return path if cur == self.id and len(path) > 1 else []

    def _should_back_off(self, r, occupant, reach, claims, key) -> bool:
        """Deterministic cycle breaker: among cycle members that HAVE a free neighbour, the lowest
        priority one backs off. Everyone computes the same answer from the same Hellos."""
        cyc = self._wait_cycle(r, occupant)
        if not cyc:
            return False
        blocked = set(occupant) | reach | set(claims) | {self.node}
        node_of = lambda rid: self.node if rid == self.id else self.peers[rid].hello.node
        key_of = lambda rid: key if rid == self.id else self._prio(rid)
        movable = [x for x in cyc if any(m not in blocked for m in self.rm.neighbors(node_of(x)))]
        return bool(movable) and min(movable, key=key_of) == self.id

    def _swap_partner(self, occupant: dict[int, int]) -> int | None:
        """Fresh peer B such that I want B's node and B wants mine (PIBT alone cannot resolve
        this on dead ends and single-lane spurs)."""
        nxt = self.guide[1] if len(self.guide) > 1 and self.guide[0] == self.node else None
        if nxt is None or nxt not in occupant:
            return None
        b = occupant[nxt]
        return b if self._wants(b) == self.node else None

    def _swap_candidates(self, b, key, occupant, reach, obstacles) -> tuple[list[int], str]:
        pv = self.peers[b]
        bn = pv.hello.node
        b_path = set(pv.intent[: self.cfg.horizon + 1]) if pv.intent and pv.intent[0] == bn else {bn}
        my_path = set(self.guide[: self.cfg.horizon + 1])
        busy = set(occupant) | reach | obstacles
        ok = lambda x, partner: x != partner and x not in busy
        mine_side = [x for x in self.rm.neighbors(self.node) if ok(x, bn) and x not in b_path]
        b_side = [x for x in self.rm.neighbors(bn) if ok(x, self.node) and x not in my_path]
        branchy = lambda x: (not self.rm.is_turnout(x), len(self.rm.neighbors(x)) < 3, x)
        if mine_side and (not b_side or key < self._prio(b)):
            return sorted(mine_side, key=branchy) + [self.node], f"swap: side-step for R{b}"
        if b_side:
            return [self.node], f"swap: R{b} side-steps"
        mine_back = [x for x in self.rm.neighbors(self.node) if ok(x, bn)]
        b_back = [x for x in self.rm.neighbors(bn) if ok(x, self.node)]
        if mine_back and (not b_back or key < self._prio(b)):
            return sorted(mine_back, key=branchy) + [self.node], f"swap: retreat for R{b}"
        return [self.node], f"swap: R{b} retreats"

    # ---- yield mode: get off the partner's path, then stay clear until it has passed ---------
    def _bpath(self, b: int) -> list[int]:
        pv = self.peers[b]
        if pv.intent and pv.intent[0] == pv.hello.node:
            return pv.intent[: self.cfg.horizon + 1]
        return [pv.hello.node]

    def _active_yield(self, r: int) -> int | None:
        best = None
        for b, until in list(self.yield_to.items()):
            pv = self.peers.get(b)
            if pv is None or not self._fresh(pv, r) or until < self.tick:
                del self.yield_to[b]
                continue
            bpath = self._bpath(b)
            near = self.node in bpath or any(n in bpath[1:] for n in self.rm.neighbors(self.node))
            lower = self.cfg.yield_only_higher and self.bkey is not None and self._prio(b) < self.bkey
            if not near or lower:
                del self.yield_to[b]
                continue
            if best is None or self._prio(b) > self._prio(best):
                best = b
        return best

    def _yield_candidates(self, b: int, occupant, reach, obstacles) -> tuple[list[int] | None, set[int]]:
        """(candidates, nodes to avoid). On B's path: side-step off it (pockets first), else retreat
        away from B. Off B's path: normal candidates, but never step back onto B's path."""
        bpath = self._bpath(b)
        bset = set(bpath)
        if self.node not in bset:
            return None, bset
        bnode = self.peers[b].hello.node
        busy = set(occupant) | reach | obstacles
        free = [x for x in self.rm.neighbors(self.node) if x not in busy and x != bnode]
        side = sorted((x for x in free if x not in bset),
                      key=lambda x: (not self.rm.is_turnout(x), len(self.rm.neighbors(x)) < 3, x))
        d0 = self.rm.dist(self.node, bnode)
        back = sorted((x for x in free if x in bset and self.rm.dist(x, bnode) > d0),
                      key=lambda x: (not self.rm.is_turnout(x), -len(self.rm.neighbors(x)), x))
        return side + back + [self.node], set()


    def _idle_move(self, r, occupant, reach, claims, obstacles) -> tuple[int | None, str]:
        """Idle (or parked at goal) robots get out of the way: step aside when pushed, or when a
        moving robot's broadcast path runs through my node within the next two steps."""
        threat = None
        for rid, pv in self.peers.items():
            if not self._fresh(pv, r):
                continue
            if pv.hello.claim == self.node or self.node in self._bpath(rid)[1:3]:
                if threat is None or self._prio(rid) > self._prio(threat):
                    threat = rid
        if threat is None:
            return None, ""
        tp = set(self._bpath(threat)) | {self.peers[threat].hello.node}
        free = [x for x in self.rm.neighbors(self.node)
                if x not in occupant and x not in reach and x not in obstacles and x not in claims]
        free.sort(key=lambda x: (x in tp, not self.rm.is_turnout(x), -len(self.rm.neighbors(x)), x))
        if free:
            return free[0], f"idle: clearing path for R{threat}"
        # push chain: ask an idle neighbour (lower priority than my threat) to make room
        idle_nb = [x for x in self.rm.neighbors(self.node)
                   if x in occupant and x not in obstacles and x != self.peers[threat].hello.node
                   and self.peers[occupant[x]].hello.goal is None and x not in claims]
        idle_nb.sort(key=lambda x: (x in tp, x))
        if idle_nb:
            return idle_nb[0], f"idle: asking R{occupant[idle_nb[0]]} to make room for R{threat}"
        return None, f"idle: cannot clear for R{threat}"


    # ---- claim round ------------------------------------------------------------------------
    def claim_phase(self, tick: int) -> tuple[Hello, Intent, Lease | None, list[str]]:
        self.tick = t = tick
        r = claim_round(t)
        if self.boot_tick is None:
            self.boot_tick = t
        events: list[str] = []
        obstacles = self._live_obstacles()
        for rid in [x for x, pv in self.peers.items() if r - pv.heard_round > 2 * self.cfg.forget_after]:
            del self.peers[rid]
            self.roster.discard(rid)
            events.append(f"forget R{rid}: silent {self.cfg.forget_after} ticks")
        occupant, reach, claims = self._views(r, r - 1)
        key = self.bkey if self.bkey is not None else self.priority_key()
        new_claim: int | None = None
        pusher: int | None = None
        why = ""
        if self.goal is not None and self.node != self.goal:
            # follow the committed guide path: drop the step just taken or the wait just done
            g = self.guide
            if g and len(g) > 1 and ((self.moved_last and g[0] == self.prev_node and g[1] == self.node)
                                     or (not self.moved_last and g[0] == self.node and g[1] == self.node)):
                g = g[1:]
            valid = bool(g) and g[0] == self.node and len(g) > 1 and g[1] not in obstacles and g[1] not in reach
            self.blocked_ticks = self.blocked_ticks + 1 if (valid and not self.moved_last and g[1] != self.node) else 0
            res = ReservationTable(self.cfg.horizon)
            for rid, pv in self.peers.items():
                if self._fresh(pv, r) and self._prio(rid) > key and pv.intent and pv.intent[0] == pv.hello.node:
                    res.add_path(pv.intent, t)
                elif self._fresh(pv, r):
                    k = self.cfg.lower_prio_reserve
                    if k is None:
                        res.add_path([pv.hello.node], t)
                    else:
                        res.add_path([pv.hello.node] * k, t, tail=False)
                else:
                    for n in self._reach(pv, r - 1):
                        res.add_path([n], t)
            cand = plan_guide_path(self.rm, self.node, self.goal, res, t, self.cfg.horizon, obstacles)
            self.metrics["replans"] += 1
            if not valid or self.blocked_ticks >= self.cfg.patience or len(cand) + self.cfg.switch_margin < len(g):
                if valid and self.blocked_ticks >= self.cfg.patience:
                    events.append(f"replan: blocked {self.blocked_ticks} ticks at {self.node}")
                g, self.blocked_ticks = cand, 0
            self.guide = g

            # clear the mouth: if I stand on a corridor mouth and a robot inside will exit through
            # my node, get off its path now instead of waiting in the doorway
            for rid, pv in (self.peers.items() if self.cfg.clear_mouth else ()):
                if not self._fresh(pv, r) or rid in self.yield_to:
                    continue
                pc = self.rm.corridor_of(pv.hello.node)
                if pc is None or self.rm.corridor_of(self.node) == pc or self.node not in self.rm.mouths[pc]:
                    continue
                bp = self._bpath(rid)
                if self.node in bp[1:]:
                    self.yield_to[rid] = t + 2 * self.cfg.horizon
                    events.append(f"clear mouth for R{rid}")
            k_ex = self.cfg.exit_yield_within
            if k_ex:
                for rid, pv in self.peers.items():
                    if not self._fresh(pv, r) or rid in self.yield_to:
                        continue
                    pc = self.rm.corridor_of(pv.hello.node)
                    if pc is None or self.rm.corridor_of(self.node) == pc:
                        continue
                    bp = self._bpath(rid)
                    if self.node in bp[1:k_ex + 1] and self._peer_dir(rid, pc) != 0:
                        self.yield_to[rid] = t + 2 * self.cfg.horizon
                        events.append(f"clear exit for R{rid}")
            k_py = self.cfg.proactive_yield_within
            if k_py:
                for rid, pv in self.peers.items():
                    if not self._fresh(pv, r) or self._prio(rid) <= key or rid in self.yield_to:
                        continue
                    bp = self._bpath(rid)
                    if self.node in bp[1:k_py + 1] and (len(self.guide) < 2 or self.guide[1] == self.node
                                                         or self.guide[1] in bp[: k_py + 1]):
                        self.yield_to[rid] = t + 2 * self.cfg.horizon
                        events.append(f"proactive yield to R{rid}")
            pusher = self._pushed_by(r, key)
            eff_key = max(key, self._prio(pusher)) if pusher is not None else key
            cands = preference_order(self.rm, self.node, self.guide, self.goal, obstacles, self.cfg.honour_planned_wait)
            # swap takes precedence over push: both robots then run the same deterministic rule
            swap_b = self._swap_partner(occupant)
            if swap_b is not None and pusher is not None:
                pusher, eff_key = None, key
            avoid: set[int] = set()
            d_now = self.rm.dist(self.node, self.goal)
            if d_now < self.best_dist:
                self.best_dist, self.stuck = d_now, 0
            else:
                self.stuck += 1
            if self.stuck >= self.cfg.livelock_after and self.escape_left == 0:
                self.escape_left, self.stuck = self.cfg.escape_ticks, 0
                self.yield_to.clear()
                events.append(f"livelock escape: no progress for {self.cfg.livelock_after} ticks")
            if self.escape_left > 0:
                self.escape_left -= 1
                free = [m for m in self.rm.neighbors(self.node)
                        if m not in occupant and m not in reach and m not in claims and m not in obstacles]
                self.rng.shuffle(free)
                cands = (free + [self.node]) if self.rng.random() < 0.5 else ([self.node] + free)
                why = "livelock escape: random move"
            elif swap_b is not None:
                cands, why = self._swap_candidates(swap_b, key, occupant, reach, obstacles)
                events.append(why)
                if len(cands) > 1:
                    self.yield_to[swap_b] = t + 2 * self.cfg.horizon
                    self.metrics["swaps_resolved"] += 1
            elif pusher is not None:
                self.yield_to[pusher] = t + 2 * self.cfg.horizon
                ycands, avoid = self._yield_candidates(pusher, occupant, reach, obstacles)
                cands = ycands if ycands is not None else cands
                why = f"pushed by R{pusher}"
            elif (ymode := self._active_yield(r)) is not None:
                ycands, avoid = self._yield_candidates(ymode, occupant, reach, obstacles)
                if ycands is not None:
                    cands, why = ycands, f"yield: clearing path of R{ymode}"
                else:
                    why = f"yield: holding clear of R{ymode}"
            elif not self.moved_last and self.claim not in (None, self.node) and self.claim in occupant                     and self._should_back_off(r, occupant, reach, claims, key):
                free = [m for m in self.rm.neighbors(self.node)
                        if m not in occupant and m not in reach and m not in claims and m not in obstacles]
                free.sort(key=lambda m: (not self.rm.is_turnout(m), self.rng.random()))
                cands = free + [self.node]
                self.metrics["cycles_broken"] += 1
                events.append("wait-for cycle: backing off")
            for m in cands:
                if m == self.node:
                    break
                if m in obstacles or m in reach or m in avoid:
                    continue
                if m in occupant and self._prio(occupant[m]) >= eff_key:
                    continue                         # cannot push a higher-priority occupant
                if m in claims and self._prio(claims[m]) > eff_key:
                    continue                         # contested by higher priority
                gate = self._corridor_gate_blocked(m, r, eff_key)
                if gate is None and self.cfg.gate_mouths:
                    gate = self._mouth_gate_blocked(m, r, eff_key)
                if gate is not None:
                    events.append(f"corridor gate: waiting for R{gate}")
                    # clear the mouth: the robot inside will exit through my node, get off its path
                    gp = self.peers[gate]
                    if gp.hello.node in self.rm.cidx.get(self.rm.corridor_of(m), {}) and self.node in self._bpath(gate):
                        self.yield_to[gate] = t + 2 * self.cfg.horizon
                    continue
                new_claim = m
                if m in occupant:
                    self.metrics["pushes"] += 1
                    events.append(f"push R{occupant[m]} off {m}")
                break
        else:
            new_claim, why = self._idle_move(r, occupant, reach, claims, obstacles)
        self.claim = new_claim
        if pusher is not None and new_claim is not None:
            self.metrics["yields"] += 1
        hello = self._hello(r, key)
        intent_nodes = self.guide[: self.cfg.horizon + 1] if self.guide and self.guide[0] == self.node else [self.node]
        intent = Intent(robot_id=self.id, tick=r, nodes=intent_nodes, priority=key)
        lease = None
        cid = self.rm.corridor_of(self.claim) if self.claim is not None else None
        if cid is not None and self.claim != self.node:
            lease = Lease(robot_id=self.id, tick=r, corridor=cid,
                          direction=self.rm.corridor_direction(cid, self.node, self.claim),
                          priority=key, expires_tick=t + self.cfg.lease_ttl)
        yld = None
        if pusher is not None and new_claim is not None:
            yld = Yield(robot_id=self.id, tick=r, to_node=new_claim, for_robot=pusher, reason="pushed")
        self._pending = (intent, lease, yld, events, why)
        return hello, intent, lease, events

    # ---- move round -------------------------------------------------------------------------
    def move_phase(self, tick: int) -> Decision:
        t = tick
        r = move_round(t)
        obstacles = self._live_obstacles()
        self.degraded = bool(self.peers) and (r - self.last_heard_any) >= 2 * self.cfg.silence_stop_after
        missing = self._missing_acks(r)
        occupant, reach, claims = self._views(r, r)
        moved, reason = False, ""
        if self.claim is not None and self.claim != self.node:
            m = self.claim
            if t - (self.boot_tick or 0) < self.cfg.boot_hold_ticks:
                reason = "hold: boot (letting peers hear me)"
            elif missing:
                reason = f"hold: waiting for acks from {', '.join(f'R{x}' for x in missing[:3])}"
            elif m in obstacles:
                reason = f"wait: {m} blocked"
            elif m in occupant:
                reason = f"push: waiting for R{occupant[m]} to vacate {m}"
            elif m in claims:
                reason = f"wait: {m} also claimed by R{claims[m]}"
            elif m in reach:
                reason = f"wait: {m} may hold an unheard robot"
            elif self.cfg.gate_recheck_in_move and (g := self._corridor_gate_blocked(m, r, self.bkey or self.priority_key(), entering_only=True)) is not None:
                reason = f"corridor gate: R{g} using it the other way"
            else:
                self.prev_node, self.node, moved = self.node, m, True
                reason = f"move -> {m}"
        if self.goal is not None and self.goal != self.node:
            self.age += 1
        if moved:
            self.wait_age = 0
            self.metrics["moves"] += 1
            self.claim = None
        elif self.goal is not None and self.goal != self.node:
            self.wait_age += 1
            self.metrics["waits"] += 1
        self.moved_last = moved
        key = self.priority_key()
        self.bkey = key
        intent, lease, yld, events, why = self._pending or (
            Intent(robot_id=self.id, tick=r, nodes=[self.node], priority=key), None, None, [], "")
        self._pending = None
        if not reason:
            reason = why or ("idle" if self.goal in (None, self.node) else "wait: no safe claim")
        return Decision(t, self.node, moved, self.claim, reason, self._hello(r, key), intent, lease, yld, events)

    def _hello(self, r: int, key: PriorityKey) -> Hello:
        x, y = self.rm.pos(self.node)
        return Hello(robot_id=self.id, tick=r, node=self.node, priority=key, battery=self.battery,
                     load=self.load, claim=self.claim, prev_node=self.prev_node, goal=self.goal,
                     task_id=self.task_id, plan_hash=hash(tuple(self.guide)) & 0xFFFF,
                     degraded=self.degraded, heard={rid: pv.hello.tick for rid, pv in self.peers.items()},
                     x=x, y=y)
