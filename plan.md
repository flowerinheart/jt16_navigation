# JT16 FAST-LIO + Nav2 development plan

V0 status: complete — current-source small-simulation baseline, frozen as Git
tag `v0`. Arena Quick3 and fresh-mapping BARN Quick3 each pass3/3; independent
delivery/Ponytail reviews pass. Full-suite and hardware acceptance are separate
pending milestones. See `docs/v0/release.md` and `docs/v0/report.md`.

## 1. Product objective

Build a reproducible XLeRobot navigation stack on ROS 2 Jazzy and Gazebo Harmonic, using the Hesai-supported JT16 software path as the foundation:

```text
Hesai JT16 / Gazebo JT16 model
  -> Hesai PointCloud2 + IMU interface
  -> FAST_LIO_Hesai
  -> LiDAR-inertial odometry + registered 3D cloud + PCD map
  -> 3D map projection to 2D OccupancyGrid
  -> saved-map localization
  -> Nav2 global planning, local control and recovery
  -> live LiDAR updates Nav2 costmaps
```

FAST-LIO is present from V0. It is the primary odometry and mapping frontend, not a later optional replacement.

The first product remains operationally simple: build a map first, save it, then restart in navigation mode. Nav2 consumes a 2D occupancy map, but the pose and source map come from FAST-LIO rather than `slam_toolbox`.

No object detector, semantic segmentation or RGB perception is required initially. Obstacles are geometric LiDAR observations. Online mapping while navigating, semantic perception and complex global relocalization are introduced only after the saved-map baseline is measured.

## 2. Authoritative starting repositories

Development starts from the already pinned upstream repositories. Vendor code is not copied into owned packages or silently rewritten.

### Runtime foundations

- `HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai`
  - Pinned commit: `2783882405484c0c852eaffc3b07b26d1fff2556`.
  - Official Hesai adaptation of FAST-LIO2 for JT16/JT32/JT128.
  - Consumes `/lidar_points` and `/lidar_imu`.
  - Requires `x/y/z/intensity/ring/timestamp`; per-point time is required for motion compensation.
  - Publishes LiDAR-inertial odometry, registered clouds and map clouds.
- `HesaiTechnology/HesaiLidar_ROS_2.0`
  - Pinned commit: `3d54078a3d770004ef30ec2cacc26673f4172b22`.
  - Official JT16-capable ROS driver and SDK integration.

### Integration references

- `Ikunio/Lidar_nav2_ws`
  - Pinned commit: `a383f3667513eecad42defb5947233f1d0649ac3`.
  - Reference architecture for LIO odometry, registered scan generation, 3D-to-2D conversion, relocalization and Nav2.
  - Reuse concepts and verified interfaces, not absolute paths, Livox-specific assumptions or unverified clock settings.
- Nav2 official, Clearpath and TurtleBot4 sources provide lifecycle, launch and parameter conventions.
- KISS-Matcher and small_gicp remain pinned runtime libraries for a later measured relocalization upgrade.

### Upstream policy

- Keep vendor repositories at their pinned commits during each benchmark series.
- Put ROS 2 Jazzy compatibility, topic normalization, TF bridging, map conversion and launch integration in owned packages whenever possible.
- A required vendor fix must be an explicit reviewed patch or pinned fork with a recorded diff.
- Preserve upstream licenses; FAST_LIO_Hesai is GPL-2.0.
- Do not replace the Hesai implementation with the generic FAST-LIO copy embedded in a reference workspace.

## 3. Target architecture

### 3.1 Mapping mode

```text
/lidar_points + /lidar_imu
  -> FAST_LIO_Hesai
  -> odom/LiDAR pose
  -> /cloud_registered and accumulated PCD
  -> ground/self/outlier filtering
  -> versioned 3D PCD map
  -> deterministic PCD-to-OccupancyGrid conversion
  -> map.yaml + map image + metadata
```

The saved artifact is a bundle, not only a PGM file:

- Raw and filtered PCD map.
- 2D occupancy map.
- JT16/IMU calibration and mount revision.
- Projection height range and occupancy parameters.
- FAST-LIO parameters and source commit.
- Map origin, resolution, frame convention and content hash.

### 3.2 Baseline navigation mode

```text
FAST_LIO_Hesai odometry
  -> odom -> base_link

saved OccupancyGrid + projected LaserScan
  -> AMCL
  -> map -> odom

map + localization + live LiDAR obstacles
  -> Nav2
  -> cmd_vel
```

AMCL is the simplest initial saved-map correction layer. FAST-LIO continues to provide smooth local LiDAR-inertial odometry; AMCL provides the global `map -> odom` correction against the saved 2D map.

Only one component may publish each TF edge:

- FAST-LIO integration owns `odom -> base_link`.
- AMCL owns `map -> odom` in the baseline.
- A future 3D relocalizer must replace AMCL's global correction publisher, never run as a second publisher for the same edge.

### 3.3 Runtime obstacle path

The registered/current JT16 cloud serves two different consumers:

- A controlled height slice is converted to `LaserScan` for AMCL and the initial Nav2 obstacle layer.
- The original 3D cloud remains available for later voxel-layer evaluation.

The 2D slice is a navigation representation, not the source SLAM algorithm.

## 4. Unified public validation suite

All released versions use the same frozen public scenes, starts, goals, exploration trajectories, seeds, timeouts and scoring code.

### 4.0 Quick-3 tuning gate

Use the frozen `integration/benchmarks/manifests/quick3.json` subset for low-cost
algorithm repair and parameter iteration before any complete release-suite run:

- BARN world 22, seed 0: early narrow-corridor regression.
- BARN world 55, seed 0: mid-route stall regression.
- BARN world 77, seed 0: late-route stall regression.

Promotion requires all three cases; a failing pilot may stop early for repair. Failures, collisions and timeouts remain
in the results. Score physical success from final Gazebo ground-truth goal
distance and collision state; record the Nav2 action result separately.

The candidate may be considered for broader evaluation only when:

- All 3 navigation tasks physically succeed.
- Process-failure rate is zero.
- Mapping, navigation and scoring artifacts exist for all three cases.

Quick-3 is a tuning and regression gate, not a release result. Passing it does
not replace BARN-30 or Arena-6, and does not automatically start those costly
runs. Also require the frozen Arena Quick-3 (`arena_quick3.json`: small_warehouse/obs5/seed1,
map2/obs5/seed0, map5/obs5/seed1), NAV150, 3/3 strict success, zero collisions and process failures.
Both small sets must use the same final source. Require explicit approval before starting the complete V0
suite. Do not change Quick-3 membership, seeds, timeout or scoring rules to make
a candidate pass.

### 4.1 BARN-30

Select 30 environments deterministically from the 300 public BARN environments before tuning:

- 10 easy.
- 10 medium.
- 10 difficult.

Store world IDs, BARN revision, checksums, Gazebo conversion revision, exploration trajectory, start/goal poses and shortest feasible path in a benchmark manifest. The selection cannot change after V0 evaluation begins.

For each environment:

- Run one fixed mapping trajectory through FAST-LIO.
- Generate the PCD and 2D occupancy artifacts.
- Restart the stack in saved-map navigation mode.
- Run five fixed navigation seeds.

```text
30 mapping runs + 30 x 5 navigation runs = 180 runs
```

### 4.2 Arena-6

Use the six public Arena-Bench dynamic-obstacle combinations:

- Three public worlds.
- Five and ten dynamic obstacles per world.
- Ten fixed seeds per combination.

```text
6 x 10 = 60 navigation runs
```

The release suite therefore contains 30 mapping runs and 210 navigation runs. Before external delivery, also run all 300 public BARN environments; the full run is reported separately and does not change the frozen release set.

### 4.3 Metrics

FAST-LIO and mapping metrics:

- Initialization success and time.
- Complete trajectory rate.
- ATE and RPE against offline Gazebo ground truth.
- Translational/yaw drift and static drift.
- Odometry update rate, latency and dropped input frames.
- PCD completeness and map surface distance where ground-truth geometry permits.
- 2D occupied-cell precision, recall, F1 and IoU after alignment.
- Mean and 95th-percentile wall distance error.
- CPU, memory and real-time factor.

Localization metrics:

- Saved-map initialization success and latency.
- Translational and yaw error.
- Localization-loss and false-convergence counts.
- `map -> odom` jump magnitude.

Navigation metrics:

- BARN Score.
- Success rate and collision-free success rate.
- Collision and timeout rates.
- Actual path / shortest feasible path.
- Time to goal and final ground-truth goal distance.
- Arena minimum/mean obstacle clearance, acceleration, jerk, curvature and roughness.

## 5. Version comparison policy

### 5.1 Fixed comparison conditions

Every version comparison uses identical:

- Vendor commits and benchmark assets unless the version explicitly evaluates a vendor update.
- Robot geometry, mass, footprint and dynamics.
- JT16 mounting pose.
- Worlds, trajectories, starts, goals, seeds and timeouts.
- Simulator physics and sensor-noise profile.
- Reset, warm-up and failure rules.
- Offline scoring code.

All crashes, initialization failures, collisions, localization failures and timeouts remain in the result set.

### 5.2 Global non-regression gates

Against the previous released version:

- Collision-free SR may not fall by more than 1 percentage point on BARN-30 or Arena-6.
- Collision rate may not increase by more than 1 percentage point.
- Complete FAST-LIO trajectory rate and saved-map localization success may not decrease.
- ATE, RPE and 95th-percentile wall error may not worsen by more than 5% relative.
- BARN Score may not fall by more than 2% relative.
- No new TF conflict, lifecycle failure, process crash or unrecoverable hang.
- CPU or memory growth above 20% requires an explicit measured benefit and approval.

### 5.3 Required improvement

Each version after V0 must meet its version-specific improvement gate and all global gates. Use paired per-run comparisons and bootstrap 95% confidence intervals. A new component without a practical measured improvement remains an experiment and does not replace the previous release.

## 6. Release ladder

### V0: Hesai official FAST-LIO + simple Nav2 baseline

V0 is the first complete product, not a sensor-only prototype.

Deliver:

- Official Hesai JT16 driver path for hardware and an interface-compatible Gazebo publisher.
- Valid per-point `ring` and `timestamp` fields; fabricated identical timestamps are not accepted for moving runs.
- FAST_LIO_Hesai built and launched on ROS 2 Jazzy with recorded compatibility changes.
- Measured JT16-to-IMU and JT16-to-`base_link` extrinsics.
- FAST-LIO odometry, registered cloud and PCD saving.
- Deterministic PCD-to-2D occupancy conversion.
- Height-controlled cloud-to-LaserScan projection.
- AMCL saved-map localization.
- Nav2 with NavFn, the approved native MPPI candidate (DWB comparison template retained), static/obstacle/inflation layers, Collision Monitor and Simple Commander.
- BARN-30/Arena-6 manifests, runners and scorers.

V0 acceptance:

- Every one of the 30 mapping runs and 210 navigation runs reaches a scored terminal state.
- No duplicate TF publisher.
- Simulated motion changes the physical robot trajectory; ground truth never moves the robot.
- The full metric bundle is archived as the baseline for V1.

V0 does not need to beat a predecessor. Its purpose is to establish the official-Hesai-based baseline.

2026-11-01 approved simulation amendment: compare the installed Nav2 MPPI controller
with DWB on the frozen quick sets before V3. Keep FAST-LIO, LiDAR, maps, actor
mission epochs, robot limits, goal tolerances and scoring unchanged. First repair
the kinematic simulator's command response. Keep DWB as the default until MPPI
has a demonstrably better collision-free scored result on the same small set;
neither action success alone nor mixed-source historical runs justify promotion.
Do not run the full suite until the small-set gate passes. This amendment does
not waive hardware calibration or wheel-dynamics acceptance.

The adoption candidate has now passed the frozen Arena Quick-3 3/3 with matching
source/runtime identities and offline metric replay. Default runners generate
that experimental MPPI configuration and archive it. The final default-entrypoint
warehouse rerun had a real lateral dynamic collision, so final Arena+BARN delivery
verification is not passed. No full-suite or V0 hardware acceptance is claimed.

Further approved simulation amendment: use lightweight LiDAR-only motion estimates
and an installed-MPPI `CriticFunction` extension to score moving surfaces against
robot trajectories at matching times. Preserve FAST-LIO, robot limits, frozen
scenes/seeds, NAV150 and strict scoring. Disable the candidate's static future-sweep
costmap layer to avoid counting all future positions as occupied now; retain real
LiDAR obstacle costs and Collision Monitor. Validate passing before arrival,
yielding, footprint orientation and scan/pose timestamp offsets in native tests.
Arena actor commands use Gazebo's existing batch pose service with the original
0.2 s motion cadence and independent 0.1 s actual-position observations; failed
commands or stale observations invalidate the task. First obtain a credible
map2 pilot and repeat, then diagnose BARN22, then freeze one final source/runtime
and rerun Arena Quick-3 and fresh-mapping BARN Quick-3. Final independent delivery
and Ponytail reviews are required; current temporal-plugin review is BLOCK for
missing credible final small-set evidence, not a release approval.

Historical measured small-simulation gate: `integration/results/v0/timed-mppi-memory-repeat/`
contains Arena Quick-3 strict 3/3 and BARN Quick-3 strict 3/3 with fresh maps,
zero collisions/input failures and valid localization/scoring. All six navigation
runs and three mapping runs share 77 source and 33 runtime hashes. Navigation
NPZ replay, TF audits, map bundle hashes and mapping diagnostics have been checked.
This closed that candidate's measured small-set gate only; final independent delivery/Ponytail
review, full-suite acceptance and hardware calibration are separate requirements.
Historical BLOCK reports and failed candidates remain unchanged.
The post-smoke-deletion `timed-mppi-clean-final` rerun passed map2 but warehouse
failed with dynamic/static collisions. Current-source small-set acceptance is open.
Diagnose and repair warehouse, validate a warehouse pilot and repeat, then freeze
and rerun both Quick-3 sets before final reviews. Do not start a full suite.
Interim `timed-mppi-distance-repeat` passes Arena3/3 and freshly mapped BARN22,
but BARN22 takes148.46s and BARN55/77 are absent. Subsequent independent mapping
exit/resume repairs change source identity, so that root is historical/interim
evidence only. Revalidate the mapping-entry fixes, then use a fresh output root
for a current-source frozen dual-Quick-3 gate and independent final reviews.
The repaired current source has now completed that measured gate in
`integration/results/v0/timed-mppi-startup-pilot/`: Arena3/3 and BARN3/3 with
fresh COMPLETE maps, zero collisions/input failures, valid localization/scoring,
matching76 source/33 runtime hashes, NPZ replay and TF/map bundle audits.
The earlier `timed-mppi-current-gate` startup PROCESS_FAILURE remains preserved;
no source/RMW/parameter change was used to hide it. BARN22 takes149.304s against
NAV150, so this is not a claim of robust timing or universal metric improvement.
New independent re-review: `docs/reviews/20261101-current-gate-rereview.md`.
Full-suite V0 release/hardware calibration remain outside this small-set gate.

### V1: FAST-LIO input integrity, timing and calibration

Improve the official baseline without changing Nav2 planner/controller selection:

- Replace instantaneous/fabricated point time with a rotating JT16 scan timing model.
- Match official Hesai PointCloud2 field names, units and timestamp semantics.
- Validate LiDAR/IMU clock offset and synchronization.
- Revalidate LiDAR-to-IMU and LiDAR-to-base extrinsics.
- Add explicit self, range and outlier filtering at the correct pipeline stage.
- Correct covariances, TF frames and initialization parameters.

Required V1 improvement over V0:

- FAST-LIO ATE or RPE improves by at least 10% relative; and
- 95th-percentile wall error improves by at least 10% or occupied-cell F1 improves by at least 2 percentage points; and
- initialization/complete-trajectory rate does not decrease; and
- all global non-regression gates pass.

### V2: 2D projection and saved-map localization quality

Keep FAST-LIO as the frontend and optimize only the map/navigation interface:

- Derive projection height from the robot footprint and obstacle coverage, not visual preference.
- Compare one slice with a small multi-band projection only if the frozen suite exposes misses.
- Tune PCD-to-grid resolution, occupancy thresholds and unknown-space handling.
- Tune AMCL motion/sensor models and initial-pose handling.
- Add localization validity monitoring and stop Nav2 when global pose is invalid.

Required V2 improvement over V1:

- AMCL initialization success improves by at least 2 percentage points or initialization latency decreases by at least 20%; and
- false convergence/localization-loss count decreases by at least 20%; and
- BARN collision-free SR improves by at least 2 percentage points or BARN Score improves by at least 5% relative; and
- all global non-regression gates pass.

### V3: Nav2 planning, control and recovery

Use V2's FAST-LIO, map and localization unchanged while improving navigation:

- Tune footprint, inflation, obstacle marking/clearing and observation persistence.
- Tune progress and goal checkers.
- Compare NavFn with Smac Planner 2D only when global planning is a measured bottleneck.
- Compare DWB with MPPI only when control is a measured bottleneck.
- Add Collision Monitor and bounded recovery behavior.
- Keep the smallest combination that wins the complete suite.

Required V3 improvement over V2:

- BARN Score improves by at least 5% relative; and
- BARN collision-free SR improves by at least 2 percentage points; and
- Arena collision-free SR does not decrease; and
- all global non-regression gates pass.

### V4: 3D obstacle use and 3D saved-map relocalization

This version is introduced only when V3 results show a limitation of the 2D interfaces.

Candidate upgrades:

- Feed filtered JT16 points to a local Nav2 voxel layer for low/high obstacle coverage.
- Use KISS-Matcher for coarse global initialization against the saved PCD.
- Use small_gicp for local PCD tracking/refinement.
- Replace AMCL's `map -> odom` publisher only after the 3D relocalizer is valid; never publish both corrections simultaneously.
- Keep the 2D OccupancyGrid for Nav2 global planning unless a demonstrated requirement justifies a different planner.

Required V4 improvement over V3:

- Arena collision rate decreases by at least 20% relative or collision-free SR improves by at least 5 percentage points; and
- saved-map localization success improves by at least 2 percentage points or false convergence decreases by at least 50%; and
- median time-to-goal does not increase by more than 10%; and
- all global non-regression gates pass.

If these improvements do not appear, retain the simpler V3 architecture.

## 7. Later experiments, outside the release ladder

### Online mapping while navigating

FAST-LIO already runs online, but using a continuously changing global map for formal Nav2 navigation changes the protocol. Evaluate it separately until it can beat the current saved-map release under the same tasks.

### Semantic perception

Add vision or learned perception only for a demonstrated geometry-only failure such as glass, negative obstacles or explicit human-aware behavior. Detection accuracy alone is not sufficient; the same BARN/Arena navigation metrics must improve.

### Alternative LIO frontend

Point-LIO or another frontend may be evaluated only as an explicit challenger to the pinned Hesai FAST-LIO baseline. It must use the same inputs and suite and show a measured win. FAST-LIO remains the default unless displaced by evidence.

## 8. JT16 tilted-mount projection risk

The navigation-safe simulation candidate is ground-relative `(-0.03, 0, 0.36)m` with zero roll/pitch/yaw. It replaces the measured `0.676350 rad` bracket pitch after an eight-azimuth matrix detected only 4/8 obstacles at a 0.650000 rad intermediate candidate versus 8/8 when level. The physical bracket requires redesign/reorientation and remeasurement. This creates two different concerns:

- FAST-LIO needs the exact full 3D extrinsic and per-point timing.
- Nav2's 2D projection can mark ground/self returns as obstacles or miss low obstacles outside the chosen height band.

Risk controls:

- Transform points into `base_link` before applying height filters.
- Keep raw and filtered clouds as separate topics.
- Remove self points by robot geometry/radius, not a broad blind range alone.
- Score multiple candidate height bands offline on the frozen public scenes before choosing one.
- Record false-positive occupied cells, missed obstacle cells and range-dependent coverage.
- Use different inputs for FAST-LIO and Nav2 when necessary: do not destroy SLAM information merely to make a clean LaserScan.
- Promote a voxel layer only if it measurably reduces misses and collisions on the same suite.

## 9. Evidence retained for every version

- Vendor commits, owned source revision and all patches/diffs.
- URDF/SDF, extrinsics, timing model and driver configuration.
- Raw PointCloud2 field metadata and representative rosbag/MCAP data.
- FAST-LIO parameters, trajectories, registered clouds and PCD maps.
- 2D map conversion configuration and generated maps.
- TF graph and publisher ownership.
- Per-run ground truth, estimated trajectory, Nav2 result, collision contacts and recovery counts.
- Aggregate metrics with paired differences and 95% confidence intervals.
- CPU, memory, real-time factor and dropped-message counts.
- Representative unedited success and failure recordings.

Each version report must state what improved, by how much, what regressed and whether every promotion gate passed.

## 10. Immediate work order

1. Freeze the pinned Hesai driver and FAST_LIO_Hesai commits as the V0 vendor baseline.
2. Freeze BARN-30 and Arena-6 manifests before parameter tuning.
3. Migrate the robot, JT16 simulation and owned adapters into relocatable ROS packages.
4. Implement interface-compatible JT16 `PointCloud2` with real per-point timing and IMU synchronization.
5. Resolve the historical static FAST-LIO displacement before navigation claims.
6. Validate dynamic wheel motion, collision geometry and FAST-LIO trajectory accuracy.
7. Save a versioned PCD and generate a deterministic 2D occupancy map.
8. Integrate AMCL and NavFn with the approved native MPPI candidate; retain DWB as an explicit comparison template.
9. Run every candidate on Quick-3 and retain all success, failure, collision and timeout results.
10. Iterate on the frozen small sets until BARN Quick-3 and Arena Quick-3 each have 3/3 strict success with zero collisions/process failures on the same final source.
11. Do not expand a failing candidate to the full suite; retain every failure and replay archived metrics before promotion.
12. Report both small-set results and obtain fresh delivery/ponytail reviews, then wait for explicit approval before running BARN-30 and Arena-6.
13. After approval, run the complete V0 suite and archive the baseline before V1 changes.

## 11. Current blockers and risks

- Historical instantaneous scans did not model rotation; the current adapter assembles 10 rolling sectors from 100 Hz snapshots with per-point timestamps. Manufacturer timing and channel calibration remain unvalidated.
- Historical static FAST-LIO output showed approximately 0.225 m displacement; precision is not accepted.
- Wheel dynamics and reliable collision primitives are not yet validated.
- The original mount attitude was measured, but the new horizontal candidate and retained translation have not been remeasured on hardware.
- The current ideal beam elevations are not manufacturer channel calibration.
- The original inclined scan missed a low front obstacle. The corrected provisional beam sign, 0.650000 rad candidate and self-masked virtual scan recover it in the fixed static scene, but broader dynamic coverage is not accepted.
- FAST_LIO_Hesai targets ROS 2 Humble upstream; Jazzy compatibility must be verified and isolated in explicit changes.
- BARN/Arena require a versioned Gazebo Harmonic adaptation; adapted results must not be represented as the official Jackal leaderboard.
- Historical static results under `integration/` are useful evidence but do not constitute a dynamic V0 pass.
