# The stop-and-wait baseline, and why the comparison is fair

The problem statement's success criterion compares against "traditional stop-and-wait methods
when handling overlapping paths". The baseline in `baseline/stop_and_wait.py` is a conventional
centralized traffic manager. It is deliberately stronger than a naive one, so a win against it
means something.

## What the baseline gets for free

- **Perfect, instant information.** A central manager sees every robot every tick. No packet loss
  applies, even in the "30% loss" scenarios, where only SANCHAR suffers the loss.
- **Directional zone control.** A single-lane stretch between passing places is one zone. Robots
  heading the same way may share it as a convoy. Classic AGV control allows one vehicle per zone.
- **Clear-the-mouth.** A robot waiting to enter a zone steps aside if opposing traffic is coming
  out, instead of blocking the exit.
- **FCFS service.** The longest-waiting robot is served first, so nothing starves.
- **Randomized timeout backoff.** A robot stuck for 5 s steps off the blocker's path, holds for 2
  to 6 s, and then replans.
- **Idle nudging and central dispatch.** In live-order mode the oldest open order goes to the
  nearest idle robot, instantly. Idle robots blocking a working robot are moved aside.
- **Same physics as SANCHAR.** It never enters a node that is occupied at the start of the tick.

## What makes it stop-and-wait

- A robot follows its shortest static path. It stops when the next node is occupied, reserved,
  blocked, or behind opposing zone traffic.
- There is no re-routing around robots or blocked aisles, no negotiation and no pushing.
- In a Wi-Fi dead zone a robot loses its link to the manager and safety-stops. That is standard
  AMR behaviour.

## How results are reported

- 30 seeds per scenario family. Both policies see the identical instance: starts, orders and
  chaos events.
- Two reduction figures per family. "All seeds" charges gridlocked baseline runs at the time cap.
  "Both finished" drops those runs. The deck quotes the "both finished" figure, which is the more
  conservative of the two.
- The baseline's gridlock count is reported separately.
- The weak case is reported. At an isolated two-way chokepoint with two robots per direction, the
  baseline's perfect-information zone control finishes about 16% sooner than SANCHAR.
