What this change does:

The stack uses Hesai FAST-LIO for odometry, AMCL for map correction, and Nav2 MPPI for navigation. A LiDAR-only tracker adds short-horizon forecasts to the local costmap. This review treats all untracked owned source as new code and assumes one local ROS/Gazebo task at a time.

## Must fix

1. **Future occupancy loses its time** (`integration/scripts/motion_prediction.py:101`, `src/jt16_bringup/src/prediction_layer.cpp:95`)

   - **What this is:** The tracker shifts measured surfaces through the next four seconds. The layer puts every shifted surface into the same present-time map.
   - **Problem:** A robot that will leave before an obstacle arrives is still penalized for the future overlap. Conversely, a stationary robot can be hit by a lateral mover. The current same-source BARN22 run times out at 6.11 m from the goal; recorded map2 candidates also have real dynamic collisions with valid localization and scoring.
   - **Fix:** Use the approved native MPPI critic extension to compare robot and obstacle positions at the same future time. Keep current LiDAR collision checks, remove the duplicate static future penalty from that candidate, and test both yielding and passing before arrival.
   - **If we skip it:** Further static-cost tuning can trade a narrow-corridor stall for a dynamic collision. The two small sets remain unaccepted.

2. **A failed actor move can still look like a valid benchmark** (`integration/scripts/sim_state_adapter.py:51`)

   - **What this is:** Arena actors move through checked SetEntityState calls. Independent GetEntityState polling records actual positions.
   - **Problem:** `observe_obstacle` logs a failed move, releases its pending flag, and continues. Polling can still produce fresh observations for the unmoved actor. Those observations pass the timestamp-based scorer, although the requested actor motion failed. The existing failed-move test checks only the log and pending flag.
   - **Fix:** Raise a clear error on a failed move so the adapter stops rather than quietly changing the benchmark. Keep the original error log; change the regression to require the error and no follow-up observation request. Do not change actor paths, speeds, or scoring.
   - **If we skip it:** A service failure can make an obstacle easier to avoid without marking the run as a failed benchmark execution.

## Should fix

3. **The installed-node test mixes its verification stages** (`integration/tests/ros_interface_smoke.py:40`)

   - **What this is:** `main` launches nodes, checks installed interfaces, checks derived velocity, and tests moving forecasts.
   - **Problem:** Lizard reports CC27 and radon without assertions reports CC22. The earlier complexity check omitted the tests directory, so it did not catch this remaining delivery blocker.
   - **Fix:** Move the existing velocity and moving-forecast stages into two named checks. Preserve every assertion, publication cadence, timeout, and cleanup. Include owned tests in the next complexity check.
   - **If we skip it:** The delivery quality gate still fails, and new interface checks keep growing one multi-job function.

Verdict: fix 1, 2 and 3 first.

Checked: owned source, connected runners/configuration/install rules, geometry and scoring tests, ROS interface test, native prediction layer test, README, and current source-specific benchmark records. The independent delivery review is `20261101-current-stack-prerepair-review.md`; it is BLOCK, not a release approval.

Not checked: full benchmark, hardware, or jscpd. The current navigation evidence does not establish safe hardware operation.
