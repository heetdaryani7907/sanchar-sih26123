# Safety and liveness argument for the SANCHAR coordinator

Scope: node-level coordination in `sanchar/robot/coordinator.py` (one instance per robot). The
continuous execution layer and the robot's own safety scanner sit below this and are not needed
for the argument, but they are the physical backstop, as on every commercial AMR.

## Model

- Roadmap graph. One robot per node. One hop per motion tick (1 m at 1 m/s).
- Two radio rounds per tick. In the **claim round** (round `2t`) every robot broadcasts
  `Hello(node, claim)`, where `claim` is the node it wants to enter. In the **move round**
  (`2t+1`) it enters that node or stays, then broadcasts `Hello(node)`. Real robots run the radio
  at 10 Hz, so two rounds per 1 s move is conservative.
- Messages can be lost, delayed by a round (relays), duplicated or reordered. They cannot be
  forged: every packet is HMAC-signed with the fleet key (per-robot Ed25519 planned) and passes an
  anti-replay window.
- Short-range robot-to-robot links (UWB, 802.11 ad-hoc or ESP-NOW, about 3 m) work inside Wi-Fi
  dead zones. Infrastructure links do not.
- Priority is a strict total order computed identically by every robot from broadcast state:
  `(carrying-or-starved, task age, -battery bucket, -robot id)`.

## Safety: two robots never occupy the same node, and never swap through each other

A robot A enters node `m` in move round `2t+1` only if all of the following hold.

1. `m` was A's claim in its round-`2t` Hello, so every robot that heard A knows it.
2. **Acknowledgement rule.** Every roster peer that A has not excluded has recently confirmed
   hearing A. Peer B's latest Hello lists `heard[A] >= 2t+1 - 2*ack_window`.
3. For every peer B that A knows:
   - **Fresh** (A holds B's round-`2t` Hello): `B.node != m` and `B.claim != m`. In this round B
     either stays at `B.node` or enters `B.claim`, so B cannot end at `m`. A swap would need
     `B.claim == A.node`, which B's own check forbids because `A.node` is occupied.
   - **Stale** (A's newest information on B is older): `m` is outside `reach(B)` (below).
4. `m` is not an obstacle that A knows or senses.

The rule is symmetric: if B also tries to enter `m`, B's check fails on A's claim (fresh case) or
on A's reach (stale case). Losing packets can only make a robot wait.

### Why `reach(B)` is a correct over-approximation

B moves at most once per move round, and only into a node it claimed. B can move in round `m`
only while it holds A's acknowledgement `heard[B] >= m - 2*ack_window`. A's Hellos never
acknowledge more than the newest B Hello that A holds (round `h`). So B's last possible move is
at round `h + 2*ack_window + 1`, and `reach(B)` is the ball around B's last known position whose
radius is the number of move rounds left in that budget. If A last heard B's claim, the first of
those moves can only go to that claimed node. A physical sighting of B, from A's own sensors or
relayed in a `Seen` message, resets the centre of the ball to where B was seen.

The budget lasts until B could drop A from its ack set. B drops a peer only after
`exclude_after` ticks of not hearing it, and only while B is still hearing other robots. A robot
that hears nobody assumes it is the one cut off and keeps waiting for acknowledgements. After that
point, B is either:

- far away, beyond short-range radio reach of every robot around A's next node, so it cannot
  reach that node without being heard first; or
- dead, in which case its body is caught by A's own proximity sensing before A drives into it.

A sighted B keeps a one-hop ball around the sighting.

**Residual assumption.** This part of the argument assumes that two robots from different network
islands that come within short-range reach hear each other before they come within two hops. That
holds for lossless short links. With heavy loss on the short link at exactly that moment, the
robot's safety scanner is the backstop. The benchmark never partitions the network. The dashboard
demonstrates partitions at 0% loss. Everything else is covered by the argument above, and the
property tests check it on every run.

## Liveness: every robot with a goal eventually reaches it

Mechanisms, in the order the claim round applies them:

0. **Same-round priority comparison.** Contests compare the priority keys robots actually
   broadcast. Comparing a fresh local key with a stale remote one lets both sides think they win,
   which livelocks.
1. **Total order with aging.** Task age rises every tick until the goal is reached. A robot that
   waits `w_max` ticks enters the top priority class, so nothing starves.
2. **Guide path with commitment.** Each robot plans a windowed space-time A* guide path around
   higher-priority robots' broadcast plans. It sticks to that path unless it has been blocked for
   `patience` ticks or finds a path at least `switch_margin` ticks shorter. This prevents
   oscillation between near-equal plans.
3. **PIBT push with priority inheritance.** A claim on an occupied node asks its occupant to
   vacate. A pushed robot inherits the pusher's priority and moves away from it, preferring
   lay-by pockets.
4. **Swap resolution (PIBT+swap).** When A wants B's node and B wants A's, which PIBT alone
   cannot solve on dead ends, both apply the same deterministic rule. A robot that can step off
   the other's path does so, the lower priority one if both can. Otherwise the one that can
   retreat does. Swap takes precedence over push, so the two robots never read the same standoff
   differently.
5. **Yield mode.** A robot that stepped aside keeps clearing the partner's broadcast path. It
   does not step back onto that path until the partner has passed.
6. **Corridor gate and mouth gate.** Each single-lane corridor carries one traffic direction at a
   time; same-direction convoys may share it. Corridors span lay-by pockets, so a narrow passage
   with a pocket counts as one corridor. A robot waits one node before the mouth rather than in
   the doorway, and a robot leaving a corridor outranks one blocking its exit.
7. **Wait-for cycle breaker.** Among members of a local wait-for cycle that have a free
   neighbour, the lowest-priority one backs off. Every robot computes the same member.
8. **Idle robots clear the way.** Robots without a task step off anyone's path. They push other
   idle robots along if boxed in, and leave stations that an active order needs.
9. **Randomized escape (last resort).** A robot that has not got closer to its goal for
   `livelock_after` ticks drops its yields and makes random safe moves for a few ticks.
   Randomization breaks any residual symmetric livelock with probability 1. The safety rule still
   applies to every one of these moves.

Task-level liveness comes from the market (`sanchar/robot/allocator.py`). An order whose route or
pickup is blocked is released and re-auctioned. An order whose owner falls silent is re-opened by
any robot. A blocked drop-off station is redirected to the nearest free station.

## What is checked mechanically

- `tests/test_safety.py`: no two robots share a node or swap, over 32 combinations of layout,
  fleet size (3 to 8), packet loss (0 to 50%) and seed. Every robot also makes progress.
- `tests/test_engine_and_market.py`: the live-order loop completes with zero collisions under a
  blocked aisle, a robot crash and a Wi-Fi dead zone.
- `tests/test_allocator.py`: property tests prove the task-record merge is commutative,
  associative and idempotent, so any gossip order converges.
- `bench/run_bench.py`: 30 seeds per scenario family. Every SANCHAR run is checked for
  collisions, swaps and obstacle hits by the engine, which never trusts the policy.
- Planned: `formal/Lease.tla`, a TLC model check of the claim, acknowledgement and priority rules
  for three robots on a corridor with message loss. It would check the `MutualExclusion`
  invariant and the `EventualProgress` property.
