# Navigation interface ownership

Current production flow: timestamped LiDAR/IMU → rolling acquisition cloud → FAST-LIO → base odometry/TF → deskewed scan → current surfaces/forecast surfaces → global route and local MPPI → velocity smoother → raw-scan collision monitor → simulator command.

| Contract | Existing owner | Required meaning |
|---|---|---|
| Acquisition epoch | `instant_cloud_adapter.py` | Absolute point times; header matches first retained point for HESAI preprocessing. Filtering must not shift reconstructed times. Empty frames retain original header. |
| Deskew/state epoch | FAST-LIO | IMU and scan use one acquisition clock; published body cloud and odometry refer to scan end. |
| Base pose and twist | `lio_odom_bridge.py` | Mount transform converts IMU pose to base pose; body twist is computed at odometry epochs; one odom→base TF publisher. |
| Map alignment | AMCL | Owns map→odom; covariance and transform age are checked separately from odometry. |
| Current scan | `project_nav_scan.py` | Deskewed points expressed in base_link at scan-end epoch; scan is the collision monitor's raw observation input. |
| Moving observations | `predict_obstacles.py` / `MotionPredictor` | Current confirmed surfaces remain distinguishable from extrapolated surfaces. |
| Unknown/unmapped surfaces | `predict_obstacles.py` | Existing map-exclusion output is not evidence of motion; its effect on SLAM rejection and forecasts needs fixed-input review. |
| Forecast risk | PredictionLayer / TimedObstacleCritic | Current positions, velocity and epoch are separate from future occupancy. A constant-velocity forecast is not certainty about future turns. |
| Static footprint cost | Shared Nav2 raster/Smac correction | Production planner/controller use compatible footprint raster and published pose conventions. |
| Startup/runtime input validity | `run_nav2_goal.py` | Same checks, existing5s startup budget; invalid data never becomes a scored navigation success. |
| Execution safety | Existing Nav2 collision monitor | Uses /scan and published footprint after smoothing; approach behavior does not provide a moving-actor future oracle. |

Implemented in this diagnosis: shared deterministic arena motion extraction with last-commanded velocity telemetry; common startup/runtime validity checks; rolling acquisition epoch correction. Evaluation references live only under `integration/diagnostics`; production has no ground-truth mode or world/seed branch.

Next architecture changes require recorded evidence: distinguish unknown surfaces from dynamic rejection; represent forecast uncertainty at the existing forecast interface; measure each stage's timing budget. Do not add a second pose compensator, tracker, collision validator or recovery state machine without a demonstrated failure at that boundary.

Validation order: constructive fixed-input regression → normal ROS wiring smoke → fresh Dev16 gate → prescribed repeats/full evaluation. Source/config/runtime inputs remain frozen within each batch. See [diagnosis results](diagnosis-results.md) for the54-run observations and their limits.
