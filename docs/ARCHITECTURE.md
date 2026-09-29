# SANCHAR architecture

SANCHAR (Sanskrit/Hindi *sañcāra*, "movement, communication") is a masterless coordination stack
for autonomous mobile robots. Every robot runs the same node. There is no fleet server, no leader
election and no broker. The dashboard only listens.

## One robot = one edge node

```
┌──────────────────────── one AMR (Raspberry Pi 4/5, Jetson, or one OS process in sim) ────────────────────────┐
│ Sensing        pose, battery, proximity (blocked cells, robots within line of sight)                       │
│ Radio          signed broadcasts, 2 rounds per motion tick (10 Hz on hardware); relays neighbours' Hellos   │
│ Peer view      newest Hello / Intent / sighting per peer; acknowledgements; obstacle reports with expiry     │
│ Order market   bid → claim → pick → deliver; release on blocked route; re-open orders of silent robots      │
│ Planner        windowed space-time A* guide path around higher-priority robots' broadcast plans             │
│ Coordinator    decentralized Guided-PIBT: push, swap, yield, corridor and mouth gates, wait-for breaker     │
│ Safety rule    enter a node only if: I claimed it last round, every peer acked me recently, no fresh claim  │
│                or position conflicts, and it is outside every stale peer's reach set                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

| Module | File |
|---|---|
| Wire schemas, signing, anti-replay | `sanchar/protocol/messages.py` |
| Warehouse graph, corridors, pockets | `sanchar/world/roadmap.py` |
| Guide-path planner, reservations, PIBT candidate order | `sanchar/robot/planner.py` |
| Coordinator (claim round, move round) | `sanchar/robot/coordinator.py` |
| Order market (auction + CRDT) | `sanchar/robot/allocator.py` |
| Simulation engine (physics, radio model, sensing) | `sanchar/sim/engine.py` |
| SANCHAR over a lossy signed bus with relays | `sanchar/sim/policies.py` |
| Live orders, battery, centralized baseline dispatcher | `sanchar/sim/market.py` |
| Stop-and-wait baseline | `baseline/stop_and_wait.py` |
| Dashboard | `sanchar/dashboard/server.py`, `sanchar/dashboard/web/index.html` |
| Benchmark | `bench/run_bench.py`, `bench/scenarios.py`, `bench/market_scenarios.py` |

## Round model

Robots move 1 m/s on a 1 m roadmap, so one motion tick is one second. The radio runs faster. In
the simulator each tick has two rounds; on hardware the Hello rate is 10 Hz.

1. **Claim round.** Update the order market and take a physical sensing sweep. Replan the guide
   path. Pick a claim: the next node, a push request on an occupied node, a side-step, or a wait.
   Broadcast `Hello(node, claim, priority, battery, heard)`, `Intent(path)`, sightings, and any bids
   or task records.
2. **Move round.** Check the safety rule against the claims just heard. Enter the claimed node or
   wait. Broadcast `Hello(node)`.

## Protocol

All messages are msgspec structs, encoded as MessagePack, inside an
`Envelope(ver, sender, seq, hlc, hops, body, sig)`; `hlc` carries the sender's round counter, not a
synchronized clock. The envelope is HMAC-SHA256 signed with a fleet key; per-robot Ed25519 keys are
planned. It is
checked against a per-sender sliding anti-replay window, so a relayed copy of a message already
heard is dropped.

| Message | Sent | Purpose |
|---|---|---|
| `Hello` | every round | position, claim, priority, battery, load, task, goal, `heard` acknowledgement map |
| `Intent` | claim round | next 10 nodes of the guide path; others plan around higher-priority intents |
| `Seen` | claim round | robots this robot physically senses; pins silent or crashed robots |
| `ObstacleDelta` | on first sighting | blocked cells, with expiry (cooperative perception) |
| `TaskAnnounce` | on change, plus random anti-entropy | CRDT task record: version, status, owner, winning bid |
| `Bid` | idle robots, claim round | predicted cost to reach a pickup |
| `Bye` | leaving the floor, repeated | peers drop the robot immediately instead of timing out |
| `Lease`, `Yield`, `Award` | not sent | defined in the schema only; corridor use and give-way are decided from broadcast claims and Intents, and awards live in the `TaskAnnounce` record |

Design rules:

- Every message carries the sender's current state, so a lost packet is repaired by the next one.
  There are no request/response exchanges, no acknowledgements at the transport layer, and no
  leader.
- Robots re-broadcast neighbours' Hellos for up to two hops, so news crosses dead zones through
  whoever can hear.
- Short-range robot-to-robot links, about 3 m, work even where Wi-Fi does not.
- Measured traffic is about 2 to 4 KB per robot per second, relays included.

## Coordination core

Guided-PIBT is the family that won the 2023 League of Robot Runners (WPPL; Chen et al., AAAI
2024), made fully decentralized:

- Each robot plans its own guide path and uses it as the preference order for a one-step PIBT
  decision.
- Pushes carry priority inheritance. The swap rule resolves standoffs on dead ends and single-lane
  spurs.
- Yield mode keeps a robot that stepped aside off its partner's path until the partner passes.
- Single-lane corridors are gated to one direction at a time; convoys may share. A robot waits one
  node before the mouth, not in it.
- A deterministic wait-for cycle breaker and a randomized last-resort escape guarantee progress.

Safety and liveness are argued in `docs/DEADLOCK_FREEDOM.md`.

## Order market

- Idle robots bid their obstacle-aware ETA to a pickup. After a two-tick window, a robot whose bid
  is the best it has heard claims the order.
- Concurrent claims converge through the merge rule: higher version, then more progress, then the
  lower (cost, robot id). The rule is commutative, associative and idempotent, which the property
  tests prove.
- Blocked routes release the order, and everyone re-bids. Silent owners' orders are re-opened
  after 25 s. Blocked drop stations are redirected to the nearest free station.
- Below 20% battery a robot stops bidding and goes to charge.

## Deployment path

- **Simulator, today.** One process runs N independent coordinators over a modelled radio: loss,
  dead zones, partitions and relays.
- **Multi-process, next.** One OS process per robot using UDP multicast on localhost or a LAN, so
  a robot can be killed with `kill -9` during the demo.
- **Hardware.** The same `Coordinator` and `TaskMarket` classes on a Raspberry Pi or Jetson.
  Radio over 802.11 ad-hoc or ESP-NOW; pose from the robot's own localization; optional ROS 2
  bridge through rmw_zenoh.
