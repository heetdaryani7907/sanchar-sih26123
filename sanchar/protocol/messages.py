"""Wire schemas for the SANCHAR peer-to-peer protocol.

Design rules (see docs/ARCHITECTURE.md):
* State-based and idempotent: every message carries the sender's *current* state, so a lost
  packet is repaired by the next one. No acks, no leader, no request/response.
* Small: MessagePack via msgspec, typically 40-120 bytes on the wire.
* Authenticated: every Envelope is signed (HMAC-SHA256 with a shared fleet key, 16-byte tag;
  per-robot Ed25519 keys are planned) and carries a monotonic `seq` for replay protection.
* Relayable: `hops` is decremented by each re-broadcaster (multi-hop gossip, TTL 2-3).

Priority key convention: a 5-tuple of ints where a LARGER tuple means HIGHER priority.
    (effective_load, wait_age, -remaining_path_len, -battery_bucket, -robot_id)
It is computed identically by every node from broadcast state (see robot/coordinator.py).
"""
from __future__ import annotations

import hashlib
import hmac
import os
from typing import Optional, Union

import msgspec

PriorityKey = tuple[int, int, int, int, int]
PROTOCOL_VERSION = 1


class Hello(msgspec.Struct, tag=True, omit_defaults=True):
    """Heartbeat, 5-10 Hz. Carries the one-step PIBT claim so safety needs no extra round-trip."""
    robot_id: int
    tick: int
    node: int                      # roadmap node currently occupied (or being left)
    priority: PriorityKey
    battery: float                 # 0..1
    load: bool = False             # carrying goods
    claim: Optional[int] = None    # node I intend to enter at tick+1 (None = stay)
    prev_node: Optional[int] = None  # node I am coming from (for swap-conflict checks)
    goal: Optional[int] = None
    task_id: Optional[str] = None
    plan_hash: int = 0
    degraded: bool = False         # I have not heard peers recently; I am moving conservatively
    heard: dict[int, int] = {}     # peer -> round of the newest Hello I hold from it (acknowledgements)
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class Intent(msgspec.Struct, tag=True):
    """Space-time reservation: nodes[i] is where I plan to be at tick+i (i=0 is current node)."""
    robot_id: int
    tick: int
    nodes: list[int]
    priority: PriorityKey


class Lease(msgspec.Struct, tag=True):
    """State-based claim on a single-lane corridor. Winner = highest priority among fresh claims."""
    robot_id: int
    tick: int
    corridor: int
    direction: int                 # +1 = from corridor end A to B, -1 = B to A (see Roadmap)
    priority: PriorityKey
    expires_tick: int              # TTL: a crashed holder's lease dies on its own


class TaskAnnounce(msgspec.Struct, tag=True):
    """A task record. Re-gossiped by anyone who holds it; CRDT-merged by (version, origin) order."""
    task_id: str
    pickup: int
    dropoff: int
    created_tick: int
    origin: int
    version: int = 0
    status: str = "open"           # open | awarded | picked | done
    owner: Optional[int] = None
    cost: float = 0.0              # owner's winning bid (tie-break for concurrent claims)


class Bid(msgspec.Struct, tag=True):
    task_id: str
    robot_id: int
    cost: float                    # predicted ETA (+ battery/load penalties); lower wins
    tick: int


class Award(msgspec.Struct, tag=True):
    """Deterministic result of an auction window; every node computes the same one."""
    task_id: str
    robot_id: int
    tick: int
    version: int


class ObstacleDelta(msgspec.Struct, tag=True):
    """Cooperative perception: nodes observed blocked, with expiry so stale reports decay."""
    robot_id: int
    tick: int
    blocked: list[int]
    expires_tick: int


class Yield(msgspec.Struct, tag=True):
    """Explicit retreat notice so the winner proceeds without waiting for the next Hello."""
    robot_id: int
    tick: int
    to_node: int
    for_robot: int
    reason: str = ""


class Bye(msgspec.Struct, tag=True):
    """Robot is leaving the floor (shift end, depot lift, maintenance). Repeated a few ticks."""
    robot_id: int
    tick: int
    node: int


class Seen(msgspec.Struct, tag=True):
    """Cooperative perception: robots I physically sense (LiDAR + fiducial id) and where."""
    robot_id: int
    tick: int
    seen: list[tuple[int, int]]      # (robot_id, node)


Body = Union[Hello, Intent, Lease, TaskAnnounce, Bid, Award, ObstacleDelta, Yield, Bye, Seen]


class Envelope(msgspec.Struct):
    ver: int
    sender: int
    seq: int                       # monotonic per sender; receivers drop seq <= last seen
    hlc: int                       # sender's round counter (multi-process runner; 0 in the sim); no clock sync
    hops: int                      # remaining relay hops
    body: Body
    sig: bytes = b""


_encoder = msgspec.msgpack.Encoder()
_decoder = msgspec.msgpack.Decoder(Envelope)


class Signer:
    """HMAC-SHA256 with a shared fleet key (stdlib). Per-robot Ed25519 keys are planned."""

    def __init__(self, fleet_key: bytes | None = None):
        self.key = fleet_key or os.environ.get("SANCHAR_FLEET_KEY", "sanchar-dev-key").encode()

    def sign(self, env: Envelope) -> Envelope:
        env.sig = b""
        env.sig = hmac.new(self.key, _encoder.encode(env), hashlib.sha256).digest()[:16]
        return env

    def verify(self, env: Envelope) -> bool:
        sig, env.sig = env.sig, b""
        ok = hmac.compare_digest(sig, hmac.new(self.key, _encoder.encode(env), hashlib.sha256).digest()[:16])
        env.sig = sig
        return ok


def encode(env: Envelope) -> bytes:
    return _encoder.encode(env)


def decode(data: bytes) -> Envelope:
    return _decoder.decode(data)


class ReplayGuard:
    """Per-sender sliding anti-replay window (as in IPsec/DTLS): accepts each sequence number at
    most once, tolerates reordering within `window`, rejects anything older. Relayed copies of a
    message the receiver already has are dropped here, so relays never double-deliver."""

    def __init__(self, window: int = 256):
        self.window = window
        self.top: dict[int, int] = {}
        self.seen: dict[int, set[int]] = {}

    def accept(self, sender: int, seq: int) -> bool:
        top = self.top.get(sender, -1)
        seen = self.seen.setdefault(sender, set())
        if seq <= top - self.window or seq in seen:
            return False
        seen.add(seq)
        if seq > top:
            self.top[sender] = seq
            if len(seen) > 2 * self.window:
                floor = seq - self.window
                self.seen[sender] = {x for x in seen if x > floor}
        return True
