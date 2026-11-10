# JT16 Navigation

From a clean Ubuntu 24.04 container: `bash scripts/setup.sh`.
See [deployment and dependency profiles](docs/deployment.md) for ROCm, CUDA/Thor and optional benchmark setup.

XLeRobot + Hesai JT16 mapping, AMCL localization and Nav2 navigation on ROS 2 Jazzy / Gazebo Harmonic.

V1 candidate G passed all **16/16 development cases**, **10/10 fresh BARN maps**, and **6/6 required repeats**, with zero collisions or input/process failures. Its frozen full benchmark is paused by user: 30/30 valid maps, BARN 144/150 strict successes, Arena 10/10 collected strict successes, zero collisions and one navigation PROCESS_FAILURE. Fifty Arena results and supplemental TIMEOUT reruns remain pending. V1 performance acceptance remains pending. Earlier candidates and every failed run remain preserved.
See the [V1 plan](docs/v1/plan.md) and [current acceptance report](docs/v1/report.md).
Versioned map images and success/failure replays: [evaluation demos](demos/README.md).


The [V0 release](docs/v0/release.md) and [V0 report](docs/v0/report.md) remain preserved. Its full-benchmark baseline scored 88/150 BARN and 34/60 Arena strict successes, with 27/30 valid saved maps.
Repository: [flowerinheart/jt16_navigation](https://github.com/flowerinheart/jt16_navigation).

## Pipeline and evaluation

- Official Hesai FAST-LIO supplies the only `/odom` and `odom → base_link`; AMCL owns `map → odom`.
- Native Smac Hybrid checks the rectangular footprint on saved maps and retains reverse paths with a penalty based on the forward/reverse speed limits. Native MPPI uses confirmed LiDAR motion forecasts and a capped proximity cost for foreground surfaces whose velocity is unresolved; the global map receives soft future occupancy for earlier route selection. SLAM filters retained tracked observations, while navigation forecasts also retain recently observed sides under the same motion confirmation and expiry rules. The timed critic reserves its collision penalty for unexpanded footprint overlap; unresolved expansion contributes bounded proximity cost. Costmaps use raw scans for clearing and current-confirmation-filtered scans for marking; localization, prediction and collision monitoring retain raw scans.
- Motion estimates register visible surfaces, preserving partial-view tracks without semantic labels. Confirmed motion expires after 0.5 s without a fresh confirmation.
- Scan poses interpolate bracketing odometry without extending input freshness. Benchmark scripts default to the installed Cyclone DDS middleware; the native `RMW_IMPLEMENTATION` environment variable remains available.
- Navigation uses LiDAR geometry and the saved map. Ground truth drives the prescribed mapping acquisition path, supplies the idealized simulation IMU input, and scores navigation; mapping is not autonomous exploration.
- Always use a new output root after source/config changes. Schema-2 records archive executed sources, parameters, runtime-library hashes and input identities. Do not tune inside a frozen batch.

V1 promotion requires Dev16 16/16, six specified repeat successes, then one frozen 30-map / 150-BARN / 60-Arena batch. Strict success requires a successful action within 150 s, valid inputs/localization/scoring, no collision and the unchanged ground-truth goal-distance condition. Real scan/costmap collision checks remain enabled.

The simulated JT16 channel layout, extrinsics, noise and wheel dynamics still need hardware calibration and validation. Historical prototypes and failed candidates are documented in [plan.md](plan.md) and the version reports.

## Validation

Run fixed-data checks before simulation:

```bash
/usr/bin/python3 -m unittest discover -s integration/tests -p 'test_*.py'
/usr/bin/python3 -m unittest discover -s docs/v1 -p 'test_*.py'
bash scripts/build.sh scaffold
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=194 colcon test --base-paths src --packages-select jt16_bringup
ROS_DOMAIN_ID=194 /usr/bin/python3 integration/tests/ros_interface_smoke.py
```

## Dependency and build management

- Git tracks only owned src/, configuration, launch code, scripts and documentation.
- vcstool imports Git repositories using exact commit IDs from dependencies.*.repos.
- external/runtime contains algorithm/driver sources; external/reference contains templates and evaluation references. external/ is ignored by Git.
- All dependency repositories are physical directories under external/. A fresh clone imports them via scripts/fetch_dependencies.sh. Vendor source remains separately versioned; tracked patches under patches/ are applied by fetch/build and checked by scripts/check_dependencies.py.
- colcon orchestrates builds. C++ / configuration packages use ament_cmake; future Python nodes use ament_python. No custom monolithic build system.
- ROS Jazzy + Gazebo Harmonic are the target. MPPI and Smac use patched, pinned Nav2 source; the remaining Nav2 and ros_gz components use system packages.
- scripts/build.sh scaffold builds only owned src. runtime adds patched Nav2 MPPI and Smac, FAST-LIO and the Hesai driver; their dependencies must be provisioned first. KISS-Matcher and small_gicp are CMake libraries, not assumed to be colcon ROS packages; future owned localization package will link separately installed pinned libraries through find_package.
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

External repositories retain their own licenses; owned scaffold is Apache-2.0. GPL FAST-LIO sources are not relicensed by this scaffold; distribution must preserve upstream licenses.

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

Local outputs follow [artifact storage rules](docs/artifact-storage.md): `~/.jt16-navigation/` stores evaluations, generated reports, demos and caches. Set `JT16_DATA_HOME` to change the root; reusable code and project specifications remain in Git.
