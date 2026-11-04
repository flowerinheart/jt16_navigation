## General navigation fixes done

Candidate G · 2026-11-04 12:50 CST. **Paused by user; full evaluation is incomplete and V1 is not released.**

### what is done

- **Map validity:** require fresh paired inputs, finite metrics and a nonempty saved bundle; clean up each run's process groups. Path completion alone no longer counts as a valid map.
- **Motion prediction:** register visible surfaces, interpolate scan-time odometry, transform forecasts at observation time and use Nav2's native unknown-cell merge. Freshness stays at 0.5 s.
- **Planning:** native [Smac 2D](https://docs.nav2.org/configuration/packages/configuring-smac-2d.html) + [MPPI](https://docs.nav2.org/configuration/packages/configuring-mppic.html), soft global forecasts and 0.75 m dynamic margin. Keep physical limits and collision checks; remove recovery motions that bypass the timed critic.
- **Execution:** default to installed [Cyclone DDS](https://cyclonedds.io/) after lifecycle response stalls. Build, ROS interfaces, 66 Python tests, 7 C++ tests and 2 report tests passed. Independent [code review](../reviews/v1-ponytail-review.md) passed. No world/seed branches or scoring changes.

### demo and data

- 2026-11-04, ROS 2 Jazzy / Gazebo Harmonic, fixed [BARN dataset](https://www.cs.utexas.edu/~xiao/BARN/BARN.html) and Arena manifests. Strict pass: successful action within 150 simulated seconds, valid inputs/localization/scoring, no collision and the unchanged goal-distance check.

| Development gate | Candidate G | Pass condition |
|---|---:|---|
| Dev16 navigation | 16/16; V0: 7/16 | All 16 strict passes |
| Fresh Dev16 maps | 10/10 | All valid |
| Specified Arena repeats | 6/6 | Each case passes three times |

All development and repeat runs had zero collisions, input failures and process failures.

- Frozen full evaluation: **30/30 map records, 160/210 navigation records**, serial, no automatic retries. Unrun records are missing observations, not measured failures or successes. Acceptance requires all records and all five targets.

| Metric | V0 full | Candidate G observed | V1 target |
|---|---:|---|---:|
| BARN strict success | 88/150 | 144/150 collected (150/150 planned) | ≥105/150 |
| Arena strict success | 34/60 | 10/10 collected (10/60 planned) | ≥42/60 |
| Arena collisions | 13/60 | 0/10 collected | ≤6/60 |
| Valid BARN maps | 27/30 | 30/30 collected | 30/30 |
| Navigation process failures | 10/210 | 1/160 collected | ≤4/210 |

- Valid-map means only: V0 has 27 valid / 3 invalid maps; G has 30 valid / 0 invalid collected maps. ATE measures aligned trajectory error; RPE measures one-second relative translation error. F1/IoU compare occupied cells with scene geometry.

| Mapping metric | V0 mean | Candidate G mean |
|---|---:|---:|
| ATE RMSE, m | 0.1607 (n=27) | 0.0539 (n=30) |
| Translation RPE, m / 1 s | 0.0460 (n=27) | 0.0232 (n=30) |
| Occupied F1 | 0.8647 (n=27) | 0.8828 (n=30) |
| Occupied IoU | 0.7682 (n=27) | 0.7920 (n=30) |

- Selected **Dev16** runs against the corresponding V0 full cases; these examples do not replace full evaluation.

![Selected Dev16 trajectories](assets/navigation-comparison.png)

![Selected fresh Dev16 maps](assets/mapping-comparison.png)

- Dev16 telemetry replay: actual footprint and observed obstacle positions, 8× playback, H.264, 14.25 s.

[arena-replay.mp4](assets/arena-replay.mp4)

[FAST-LIO](https://github.com/HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai) supplies odometry; [AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html) supplies map localization. Ground truth drives the prescribed mapping path and idealized simulation IMU, as in V0; mapping is not autonomous exploration. Obstacle prediction uses LiDAR geometry. Hardware/JT16 calibration remains unvalidated; previously viewed benchmarks do not establish independent generalization.

### next step

- Wait for the user to resume. Complete the remaining 50 Arena results, then rerun each original TIMEOUT once in a separate output; preserve all original success/failure results. Refresh evidence and repeat delivery review before V1 acceptance.

[Case counts and hashes](assets/comparison.json) · [Source/runtime audit](assets/run-identity-audit.json) · [Protocol audit](assets/protocol-audit.json) · [Failure/missing categories](assets/failures.csv) · [Demo hashes](assets/development-evidence.json) · [Tests](assets/test-evidence.json) · [Experiments](progress.md) · [Plan](plan.md)

Raw evidence: integration/results/v1/dev16-candidate-g, repeated-g, full-benchmark-20261104-g. Earlier candidates and failures remain preserved.
