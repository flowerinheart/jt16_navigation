## General navigation fixes done

Candidate G · 2026-11-06 20:32 CST. **Full evaluation completed. Performance targets not met; V1 is not released.**

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

- Frozen full evaluation: **30/30 map records, 210/210 navigation records**, serial, no automatic retries. Unrun records are missing observations, not measured failures or successes. All records are present; G fails the Arena success, collision and process-failure targets. One pause-triggered watchdog TIMEOUT has no trajectory or collision observation; its missing metrics remain explicit.

| Metric | V0 full | Candidate G observed | V1 target |
|---|---:|---|---:|
| BARN strict success | 88/150 | 144/150 collected (150/150 planned) | ≥105/150 |
| Arena strict success | 34/60 | 31/60 collected (60/60 planned) | ≥42/60 |
| Arena collisions | 13/60 | 20/60 collected | ≤6/60 |
| Valid BARN maps | 27/30 | 30/30 collected | 30/30 |
| Navigation process failures | 10/210 | 8/210 collected | ≤4/210 |

- Valid-map means only: V0 has 27 valid / 3 invalid maps; G has 30 valid / 0 invalid collected maps. ATE measures aligned trajectory error; RPE measures one-second relative translation error. F1/IoU compare occupied cells with scene geometry.

| Mapping metric | V0 mean | Candidate G mean |
|---|---:|---:|
| ATE RMSE, m | 0.1607 (n=27) | 0.0539 (n=30) |
| Translation RPE, m / 1 s | 0.0460 (n=27) | 0.0232 (n=30) |
| Occupied F1 | 0.8647 (n=27) | 0.8828 (n=30) |
| Occupied IoU | 0.7682 (n=27) | 0.7920 (n=30) |

- Supplemental TIMEOUT reruns: **2/10 strict successes**, 6 collisions. Each of the 10 original TIMEOUTs ran once in `timeout-retries-g-20261104`; originals were not replaced or aggregated by best result.

- Selected **Dev16** runs against the corresponding V0 full cases; these examples do not replace full evaluation.

Selected Dev16 trajectory image: `~/.jt16-navigation/reports/evaluation/v1/assets/navigation-comparison.png`.

Selected Dev16 map image: `~/.jt16-navigation/reports/evaluation/v1/assets/mapping-comparison.png`.

- Dev16 telemetry replay: actual footprint and observed obstacle positions, 8× playback, H.264, 14.25 s.

Replay video: `~/.jt16-navigation/reports/evaluation/v1/assets/arena-replay.mp4`.

[FAST-LIO](https://github.com/HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai) supplies odometry; [AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html) supplies map localization. Ground truth drives the prescribed mapping path and idealized simulation IMU, as in V0; mapping is not autonomous exploration. Obstacle prediction uses LiDAR geometry. Hardware/JT16 calibration remains unvalidated; previously viewed benchmarks do not establish independent generalization.

### next step

- Candidate O completed fresh Dev16: **10/10 valid maps, 13/16 strict navigation successes, one static collision, zero input/process failures**. Warehouse obs10 touched a static wall; BARN99 and BARN111 failed in the controller. All 26 run identities and orchestration matched before subsequent experiments. O cannot proceed to repeatability or full evaluation.
- Separated observation/forecast streams and exact duplicate-cell and trajectory-bound filtering have passed unit checks. Ordinary probes still fail or collide; none replace Candidate O or establish V1 acceptance. Rejected registration and parameter experiments retain their original evidence.
- Current-confirmation marking split passed **82 Python tests**, the runtime build and installed ROS smoke; Timed risk semantics passed **8 C++ tests**. Raw scans still drive localization, prediction, clearing and collision monitoring. The derived marking stream omits only newly confirmed moving returns and falls back to unchanged raw scans when prediction is unavailable.
- The marking-split warehouse probe failed at **108.964 s**, with **zero collisions/input failures**, **0.252 m dynamic clearance**, and **237 controller deadline warnings**. Its archive records 1113 derived scans, including 999 frames with omitted current-confirmed returns; maximum header gap was 0.14 s. The full-step probe reached the goal but collided. A separate diagnostic reached collision-free in 131.104 s and remains excluded from acceptance. Archive comparison exposed native raster broadphase false clears; a shared, geometry-derived dependency patch passed regression checks. The ordinary patched warehouse probe passed strictly in 77.462 s, with zero collisions/input failures and 0.612 m clearance; 271 controller deadline warnings remain. Loaded library and run identity were verified. Adjacent BARN99 and BARN111 probes failed at 31.238 s and 32.854 s with no collisions/input failures. Smac planner screening and published-pose alignment now share the raster correction and passed regression checks. A combined BARN99 probe still failed at 38.230 s with zero collisions/input failures; The finer local grid passed BARN99 (45.692 s) but failed BARN111 (54.224 s). Held-twist pose latency compensation passed native checks, including NaN/Inf with finite-math assumptions disabled for the optimizer source, and 82 integration/5 setup tests. Its ordinary BARN111 probe still failed (45.416 s), without collision/input failure and with 0.070 m clearance. Compensation was withdrawn after the ordinary failure; its source and tests remain archived. A native diagnostic also failed (50.774 s): all 15 collision batches hit at the first rollout step. Path orientation passed BARN111/warehouse but failed BARN99; finer saved-map resolution also failed. Lower native temperature passed both BARN probes but failed warehouse. A filtered native-model diagnostic showed only late-horizon footprint hits (earliest step 29), so no final validator was added. Temporary optimizer experiments were withdrawn; shared raster fixes and failure-only native logging remain. No candidate is frozen.
- Disabling the global forecast field passed map2 but collided in warehouse, so it was rejected. A configured-disabled PredictionLayer now skips unused subscriptions and map bounds work (5 C++ cases). Exact Timed-risk saturation preserves costs and passed 9 C++ cases; a representative large-cloud component benchmark improved 185.985→38.317 ms, with little change on a small cloud. With global prediction restored, warehouse passed strictly in 137.734 s, but BARN99 failed in 28.424 s without physical collision. Native logging found a mapped footprint obstruction at the first rollout step. A separate native retry-state bug was reproduced before fixing reset to clear the critic failure flag; all 16 C++ cases and 5 setup tests pass. The new BARN99 probe passed strictly in 37.776 s, but adjacent BARN111 failed in 31.454 s with a mapped footprint obstruction despite working retries. Native diagnostics did not capture early final-sequence collisions and were withdrawn; none of these results establish V1 acceptance.
- Preserve G and all later originals. Required six repeat runs and the full 30-map/210-navigation batch remain pending for a qualifying candidate; V1 is not ready to submit or release. Detailed iteration history is archived under `~/.jt16-navigation/experiments/`.

完整逐例结果、审计与测试回执归档于 `~/.jt16-navigation/reports/evaluation/v1/assets/`，不随 Git 提交。可用 [compare_results.py](compare_results.py) 重新生成；评分和目标定义见 [Plan](plan.md)。

Raw evidence: integration/results/v1/dev16-candidate-g, repeated-g, full-benchmark-20261104-g. Earlier candidates and failures remain preserved.
