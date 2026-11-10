# Arena regression: architecture and first principles

Analysis date: 2026-11-09. Read-only source/result inspection; no simulation or source/configuration changes. Distinguish frozen G from the later uncommitted stack.

## What actually declined

| Recorded Arena metric | V0 | G |
|---|---:|---:|
| Strict successes | 34/60 | 31/60 |
| Any collision | 13 | 20 |
| Static contacts | 1 | 12 |
| Dynamic contacts | 13 | 11 |
| Missing collision observations | 5 | 1 |
| Controller deadline warnings,60 logs | 197 | 627 |
| Median deadline warnings per run | 3 | 13 |

Static/dynamic contact counts overlap. Missing observations are not collision-free outcomes. Deadline counts include warmup and different run durations, so they are evidence of timing violations, not a normalized performance benchmark. Twelve V0 strict successes became G non-successes; nine previous non-successes became G successes. This is a behavioral tradeoff, not uniform degradation.

Arena performance declined, but recorded moving-obstacle contacts did not increase. The large recorded increase is contact with static walls inside dynamic scenes. Thus dynamic prediction alone is not a sufficient explanation.

## First-principles contracts

A navigation stack must keep the robot state, observed geometry and future obstacle geometry in compatible frames and epochs, preserve observability of state, and react before the safety margin is consumed.

An illustrative clearance budget is:

`available clearance > robot/state error + obstacle prediction error + closing speed × sensing/control delay + braking margin`.

This is a design budget, not a certified bound or current parameter formula. At0.5m/s,100ms delay consumes5cm, comparable to the5cm grid resolution. Neither fresh timestamps nor a collision-free rollout establishes true clearance when state/geometry is wrong.

Ego compensation gives `estimated world obstacle = estimated robot transform × body observation`. Velocity inferred across scans therefore includes changes in robot-pose error divided by the scan interval, plus orientation-error terms. A2cm change of translational error over100ms contributes0.2m/s apparent motion; it can cancel a slowly moving actor or make a static surface appear moving. A larger perception model does not automatically fix this coordinate problem.

## Frozen G: direct architectural changes to investigate

Actual archived warehouse parameters, not current defaults, show:

| Contract | V0 | G | Potential effect, not established sole cause |
|---|---|---|---|
| Near-field buffer | SlowCircle radius0.65m, speed cap0.1m/s, plus footprint approach | SlowCircle removed; footprint approach retained | Less early speed margin; relies more on motion prediction and closer-range approach detection |
| Sparse approach gate | min_points4 | min_points4 | A1–3point visible obstacle can fall below the remaining approach gate |
| Ego shift correction | enabled | disabled | Removes future-TF waiting but does not add equivalent observed-age compensation; clearance effect needs replay |
| Progress policy | movement_time_allowance60s | 10s | Legitimate yielding can trigger failure/recovery sooner |
| Global future geometry | no global prediction layer | enabled;4s rollout projected into2D soft costs | Planner loses obstacle arrival-time information; temporary future positions can discourage viable routes |
| Global planner | NavFn, allow_unknown=false | Smac2D, allow_unknown=true | Changes route and unknown-space choices; must validate footprint/clearance under actual map/pose |
| Local static-map source | absent | absent | Global static geometry is not independently present in the local master if live scan coverage is missing |

PredictionLayer costs are capped at252, so they are soft route preferences, not hard lethal occupancy. Nevertheless time collapse discards when the obstacle occupies a location. Combining it with stronger progress pressure and less early speed limiting can change yielding, detours and exposure to narrow clearances. It may help BARN while exposing errors in Arena; ablation is needed to identify contributions.

The increase in recorded static contacts, plus14/20 G collision cases with fused error above0.5m before earliest contact, makes state/geometry/protection interaction the first investigation priority. The pose statistic includes warmup and is association only. FAST-LIO runtime was unchanged between the frozen V0/G protocol snapshots; existing estimator weaknesses can be exposed by changed trajectories without an estimator code regression.

## Current stack: a separate feedback risk

The later moving-surface SLAM filter did not exist in G; it cannot explain the historical V0→G regression by itself. It does create a present dependency:

`FAST-LIO pose → world-frame scan clustering → motion confirmation → SLAM correspondence/map exclusion → next FAST-LIO pose`.

A wrong ego estimate can suppress actor motion confirmation. After the0.5s lease expires, those surfaces can re-enter matching/map insertion, increasing contamination. Conversely, false motion/map exclusion can remove useful static anchors. Both are hypotheses requiring actual correspondence/deletion traces.

The prior independent p25 review provides a concrete cancellation witness: actor world velocity approximately[0.17,-0.10]m/s while changes in fused-position error oppose it; visible groups6–17points lose motion confirmation. A recorded-scan replay with reference ego poses restored relevant moving output in66/70scans. This supports ego-motion error as a motion-estimation bottleneck in that later probe, not an exclusive initial FAST-LIO cause or a cause of every G collision. AMCL uses the same LiDAR stream and odometry, so switching to its map frame does not automatically provide independent information.

Current `unmapped_surfaces` are separately classified from confirmed velocity, but both are concatenated into SLAM rejection and forecast inputs. Being absent from the map is not proof of motion or an estimator outlier. Review their uncertainty/eligibility separately before changing filtering.

## Prediction and safety semantics

The tracker depends on separated compact multi-point surfaces, at least0.3s history and geometric motion gates; it is not complete coverage for sparse/merged/occluded observations. Current native approach min_points1 fixes one historical sparse gate, but a prior one-parameter experiment still collided.

Constant-velocity visible-surface prediction does not represent unknown future turns. Six reference-input warehouse replays measured2s center-error P95 of0.150–0.269m and4s of0.822–1.002m. These show model error despite supplied current velocity; they do not establish a collision cause or justify globally disabling forecasts.

Native approach simulates robot motion against currently observed obstacle positions. Stopping can avoid driving into a fixed obstacle, but cannot by itself make a stationary robot retreat from an approaching actor. A previous recorded contact occurred with final command already zero. More point detection alone does not remove this limitation.

The monitor's /scan is projected from FAST-LIO deskewed body points, not a completely independent raw-LiDAR sensing path. This shares timing/deskew dependencies; a global translation error does not necessarily corrupt body-frame clearance, so independence limitations must not be exaggerated into universal monitor failure.

## Priority and discriminating checks

1. Preserve V0/G receipts and replay their static contacts: actual pose, estimated pose, scan visibility, master/layer grids, footprint costs and final commands. Determine whether each contact is a missing-observation, coordinate/footprint, or execution-response failure.
2. Isolate the direct G safety/progress/route changes one at a time on fixed recordings; use common mission phase and frozen identities for subsequent navigation tests. Do not restore every V0 parameter blindly or drop static protection to pass a narrow case.
3. For the later stack, use the documented motion-cancellation window and nearby static counterexamples to test classification without sacrificing estimator anchors. Keep current observations, unknown surfaces and forecasts semantically distinct; propagate uncertainty instead of treating missing confirmation as absent risk.
4. Measure sensor→state→prediction→command latency, model and control budgets under verified GPU/CPU conditions. The G warning increase does not identify the dominant stage or prove a hardware limitation.
5. Add visual or learned perception only when recordings establish missing geometric/semantic information. Current cylinder benchmarks do not demonstrate video-model absence as the regression cause.

No single cause has yet been proven for the V0→G regression. The strongest current lead is reduced safety/progress tolerance combined with state/local-geometry errors; the later state↔motion-filter feedback and deterministic long-horizon forecasts are distinct architectural limitations. V1 remains BLOCK.

Evidence: archived `v0/full-benchmark-20261101` and `v1/full-benchmark-20261104-g` Arena results/parameters/logs; `docs/v1/assets/comparison.json`; `g-first-contact-analysis.json`; `matrix-02/reference-forecast-replay.json`; candidate-J independent review sections58–164; current predict_obstacles/motion_prediction/FAST-LIO/PredictionLayer/TimedObstacleCritic sources. Hardware isolation and warehouse reference-condition start phase remain unresolved limits.
