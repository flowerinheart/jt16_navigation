# JT16 Navigation

XLeRobot + Hesai JT16 mapping, map localization and Nav2 integration workspace.

V0 is complete as a **small-simulation baseline**: official Hesai FAST-LIO,
LiDAR-only Nav2 navigation, Arena Quick3 and freshly mapped BARN Quick3 each
strictly pass 3/3. Full-suite and hardware acceptance remain pending.
See [V0 release scope](docs/v0/release.md) and [technical report](docs/v0/report.md).
Historical sensor prototypes remain in integration and are documented in plan.md.

## Review repair validation

Run the cheap fixed-data regressions before simulation:

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -v
bash scripts/build.sh scaffold
# Optional installed-node check (source ROS first; use an unused domain):
# source /opt/ros/jazzy/setup.bash; source install/setup.bash
# ROS_DOMAIN_ID=194 /usr/bin/python3 integration/tests/ros_interface_smoke.py
```

Navigation now uses `lio_odom_bridge` as the only `/odom` and `odom -> base_link`
publisher. It transforms the FAST-LIO IMU pose using the simulation mount; AMCL
owns `map -> odom`. Navigation no longer launches GT wheel odometry or EKF.
Ground truth remains an offline scorer input and an explicitly idealized IMU
simulation input, not an independent hardware sensor validation.

Always use a new candidate output root after code/config changes. Schema-2 runs
archive executed source/config snapshots and hashes; runners reject resumes
with different identities. Historical `integration/results/v0/final-dwb-r1`
results are not a baseline for the corrected scorer. Mapping no longer erases
occupied cells around its trajectory, and unobserved map cells remain unknown.
This can expose genuine planning failures that old free-space assumptions hid.

Require frozen BARN Quick-3 and Arena Quick-3 each to achieve 3/3 physical success,
zero collisions/process failures under the same final source, before approving any full benchmark. No updated
success-rate claim is made by fixed-data tests alone. Full V0 acceptance still
requires measured base/LiDAR/IMU extrinsics and the actual JT16 channel correction
file; the current zero-pitch mount and uniform -40..0 degree channels are simulation
assumptions, not measured hardware calibration.

Arena/BARN navigation runners now generate the approved MPPI candidate from the
installed Nav2 example and the robot's DWB parameter template before launch.
`NAV2_PARAMS_FILE` remains an explicit comparison override. The actual generated
parameters are archived in each fresh run. An earlier temporal-MPPI candidate passed
frozen Arena Quick-3 and fresh-mapping BARN Quick-3, each strict 3/3, under one
source/runtime identity in `integration/results/v0/timed-mppi-memory-repeat/`.
All six navigation tasks had valid localization/scoring, no collisions and no
input failures. See `docs/reviews/20261101-controller-repair-validation.md` for
source-specific successes and preserved failures. This is a small-simulation
gate, not full-suite or hardware acceptance. After removing the obsolete smoke entry,
the fresh `timed-mppi-clean-final` rerun passed map2 but failed warehouse with
collisions. That candidate was not accepted and prompted warehouse-only diagnosis
and repair before another frozen small-set run.
An interim distance-loop optimization subsequently passed warehouse twice and
Arena Quick-3 (3/3), plus freshly mapped BARN22. Independent review then found
mapping exit/resume bugs, now repaired with regression tests. Those repairs changed
source identity again; the interim root is not current-source acceptance evidence.
The current source now passes Arena Quick-3 and freshly mapped BARN Quick-3, each
strict 3/3, in `integration/results/v0/timed-mppi-startup-pilot/` with matching
76 source / 33 runtime hashes, offline score replay and TF/map-bundle audits.
Final independent re-review is recorded separately. This is a measured small-set
gate only. The preceding `timed-mppi-current-gate` attempt stopped on a map2 DDS
lifecycle response timeout, preserved as a startup reliability limitation.
BARN22's current 149.304 s success has only 0.696 s margin against NAV150.

The current development candidate adds a native `TimedObstacleCritic` to MPPI.
It compares each candidate robot pose with LiDAR-measured moving surfaces at the
same future time; the static swept-future prediction layer is disabled for this
candidate. Confirmed motion bridges at most 0.5 s of lost confirmation without
refreshing its original timestamp. Real scan/costmap collision checks remain enabled.
Historical prerepair reviews retain their BLOCK verdicts; final delivery and
Ponytail reviews must assess the frozen version and its complete evidence. Arena actor movement now
uses Gazebo's native batch pose service; collision scoring still uses independent
actual-state observations and cancels when any actor observation becomes stale.
Neither an action reaching the goal nor a service acknowledgement alone is a
successful benchmark; the single-run exit status follows strict scored success.

## Dependency and build management

- Git tracks only owned src/, configuration, launch code, scripts and documentation.
- vcstool imports Git repositories using exact commit IDs from dependencies.*.repos.
- external/runtime contains algorithm/driver sources; external/reference contains templates and evaluation references. external/ is ignored by Git.
- All dependency repositories are physical directories under external/. A fresh clone imports them via scripts/fetch_dependencies.sh. Vendor source remains separately versioned and must not be silently modified.
- colcon orchestrates builds. C++ / configuration packages use ament_cmake; future Python nodes use ament_python. No custom monolithic build system.
- ROS Jazzy + Gazebo Harmonic are the target. Nav2/ros_gz are system packages; navigation2 source is a reference, not a duplicate build target.
- scripts/build.sh scaffold builds only owned src. runtime adds FAST-LIO and the Hesai driver; their dependencies must be provisioned first. KISS-Matcher and small_gicp are CMake libraries, not assumed to be colcon ROS packages; future owned localization package will link separately installed pinned libraries through find_package.
- Use rosdep install --from-paths src external/runtime/FAST_LIO_Hesai external/runtime/HesaiLidar_ROS_2.0 --ignore-src -r -y when system-install privileges are available. Existing integration/deps user-local extraction is a prototype fallback, not a portable installation.
- Do not recursively build all external sources: the reference workspace embeds alternative FAST-LIO, RTAB-Map and KISS-Matcher versions.
- Fixes to vendor code go to a pinned fork or explicit reviewed patch, never silent copies. Record changed source identity when scoring.

## Commands

```bash
/usr/bin/python3 scripts/check_dependencies.py
bash scripts/build.sh scaffold
# Fresh checkout only; import the dependencies needed for your work:
bash scripts/fetch_dependencies.sh runtime
bash scripts/fetch_dependencies.sh reference
bash scripts/fetch_dependencies.sh templates
```

.gitmodules is incomplete in upstream Lidar_nav2_ws. This workspace uses it as reference only. Its historical manually hydrated gitlinks are recorded in docs/sources.initial.lock.json; vcs import does not repair missing upstream submodule mappings automatically.

No commit or remote GitHub repository has been created. External repositories retain their own licenses; owned scaffold is Apache-2.0. GPL FAST-LIO sources are not relicensed by this scaffold; distribution must preserve upstream licenses.

Installed vcstool has a `vcs validate` exact-commit validation bug (UnboundLocalError); scripts/check_dependencies.py verifies local checkout hashes independently. Fresh network import has not been exercised end-to-end in this scaffold verification. Maintainer email in package.xml is an explicit placeholder, to be set before publishing.

## Consolidated directory layout

All project material now resides below this directory:
- src/: owned ROS packages.
- external/runtime/: frontend, driver and registration library sources.
- external/reference/: robot, navigation templates and evaluation references.
- integration/: owned static-sensor prototype, config, generators and historical evidence.
- integration/archive/pre-move/: previous generated model and build/install/log snapshots; historical paths retained, not runnable caches.
- docs/sources.initial.lock.json: original download identity (historical paths).

After relocation, regenerate integration/models and integration/worlds with integration/scripts/build_robot.py, gz sdf -p and integration/scripts/build_world.py. Rebuild runtime binaries before use; old ROOT_DIR/compiler paths are not valid. Raw historical evidence is preserved and must not be rewritten to pretend it was generated here.
