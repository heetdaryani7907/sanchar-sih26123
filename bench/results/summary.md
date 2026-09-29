| Scenario | Mode | Robots | Stop-and-wait makespan, median (s) | SANCHAR makespan, median (s) | Makespan reduction, all seeds | Makespan reduction, seeds where both finished | Latency or sum-of-completion reduction | Baseline gridlocked | SANCHAR unfinished | Collisions S / B | SANCHAR better |
|---|---|---|---|---|---|---|---|---|---|---|---|
| corridor_swap | fixed tasks | 4 | 220 | 246 | -9.9 ± 10.5% | -16.4 ± 5.2% (n=28) | -7.6 ± 9.6% | 2/30 | 0/30 | 0 / 0 | 5/30 |
| intersection_cross | fixed tasks | 4 | 800 | 114 | 85.7 ± 0.2% | n/a (baseline never finished) | 91.3 ± 0.2% | 30/30 | 0/30 | 0 / 0 | 30/30 |
| warehouse_overlap | fixed tasks | 6 | 189 | 145 | 29.3 ± 7.5% | 25.0 ± 4.8% (n=28) | 29.6 ± 6.8% | 2/30 | 0/30 | 0 / 0 | 30/30 |
| blocked_aisle | fixed tasks | 6 | 210 | 148 | 37.2 ± 8.7% | 29.1 ± 4.5% (n=26) | 37.8 ± 8.9% | 4/30 | 0/30 | 0 / 0 | 30/30 |
| warehouse_loss30 | fixed tasks | 6 | 189 | 164 | 19.4 ± 9.6% | 14.5 ± 7.2% (n=28) | 23.1 ± 7.4% | 2/30 | 0/30 | 0 / 0 | 25/30 |
| warehouse_12 | fixed tasks | 12 | 398 | 152 | 63.8 ± 6.9% | 55.9 ± 5.6% (n=23) | 63.3 ± 5.4% | 7/30 | 0/30 | 0 / 0 | 30/30 |
| market_warehouse | live orders | 6 | 323 | 186 | 49.2 ± 7.3% | 43.2 ± 5.4% (n=26) | 62.3 ± 5.0% | 4/30 | 0/30 | 0 / 0 | 30/30 |
| market_blocked | live orders | 6 | 338 | 190 | 51.7 ± 6.6% | 44.6 ± 3.4% (n=25) | 63.2 ± 4.5% | 5/30 | 0/30 | 0 / 0 | 30/30 |
| market_crash | live orders | 6 | 362 | 229 | 41.0 ± 6.2% | 37.8 ± 4.7% (n=28) | 46.9 ± 4.6% | 2/30 | 0/30 | 0 / 0 | 30/30 |
| market_loss30 | live orders | 6 | 323 | 195 | 45.1 ± 8.0% | 38.6 ± 5.8% (n=26) | 57.0 ± 5.8% | 4/30 | 0/30 | 0 / 0 | 30/30 |
| market_deadzone | live orders | 6 | 393 | 292 | 38.1 ± 11.1% | 22.6 ± 7.4% (n=22) | 28.9 ± 10.1% | 8/30 | 0/30 | 0 / 0 | 29/30 |
