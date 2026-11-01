# JT16 + XLeRobot simulation delivery plan

## Scope
JT16 ground-relative base_link position (-0.03, 0, 0.36)m; roll/yaw=0, pitch-up=0.676350 rad. Attitude measured by stationary IMU after bracket change; position retained without remeasurement. URDF pitch=-0.676350 (REP-103). Original vendor sources preserved. Full navigation success is not established by static checks.

## Status
- DONE: derivative upstream URDF + SDF with JT16 link / ideal IMU, model SDF validation.
- DONE: Gazebo Harmonic loads static derivative robot; point cloud and IMU bridge observed.
- DONE: 16x300 points, 10 scan samples and 143 IMU messages received; ideal gravity +9.80665 m/s², zero angular velocity.
- DONE: instantaneous PointCloud2 adapter supplies x/y/z/intensity/ring/timestamp, with identical point times; NOT rotating scan simulation.
- DONE: FAST_LIO_Hesai ROS2 builds on Jazzy with user-local pcl_ros extraction and system Python3.12; initializes map and publishes odometry.
- OPEN: static output sample displaced ~0.225 m from initialized origin; no ATE or drift acceptance. Investigate scene geometry, initial frame conventions, static scan matching and IMU synchronization before asserting precision.
- OPEN: dynamic robot wheel control and reliable collision geometry. Gazebo reports unsupported mesh collision construction; retain mesh visuals, derive validated collision primitives.
- OPEN: screenshot / recorded-demo visual inspection. Static loading only verified through sensors/logs.
- OPEN: manufacturer exact beam elevations / shell dimensions / actual mounting measurement, realistic scan motion and noise.
- OPEN: motion/turning/occlusion/IMU timestamp verification and trajectory scoring.
- OPEN: KISS-Matcher global initialization + small_gicp tracking + Nav2 interfaces.
- OPEN: benchmark selection. nav2_scenario_runner downloaded as runner candidate: odom trajectory path integration is not independent GT, contacts must be explicitly collected. Requires independent GT/SPL evaluator. No public benchmark was accepted as final this run; Habitat 2021 PointNav remains fallback.
- OPEN: fixed full task dataset, full SR/SPL, explicit pass threshold, repeated seeds, 3–5m and >=10m subgroup reports, recovery tests.

## Next steps
1. Resolve collision approximation, robot dynamics and static pose displacement.
2. Add proper rotating-scan simulator and sensor validation.
3. Bring up full navigation and choose benchmark with independent ground-truth scoring.
4. Run fixed complete dataset, never exclude failures; GT evaluation only, no online GT localization.

## Evidence
results/sensors-smoke.json, results/fastlio-build.log, results/fastlio-static.log, results/odometry-static.yaml, results/artifact-hashes.json.
No navigation success, ATE, map accuracy or full delivery PASS claimed.

Mount updated on 2026-10-30; previous static sensor / FAST-LIO evidence used the old 0.55m level mount and is historical, not validation of this new pose.

## 2026-10-30 updated-mount sensor validation
Loaded new 0.36m / 38.75-degree pitch-up model in Gazebo Harmonic. Captured 30 lidar frames (10Hz), 30 top RGB frames (640x480, 10Hz), 611 IMU samples (200Hz). Static acceleration matches gravity transformed into tilted sensor frame; angular velocity zero. Top camera is provisional at ground height 1.28m, down pitch 0.35rad, not a measured real-camera calibration. Lighting fixed after dark first capture. See results/20261030-new-mount-lit.
All 16 rings present, 2962 finite returns / 4800 rays in sampled scan. 0.15..12.48m ranges. 1302 returns within 0.5m need self-point filtering. Low box in front is visible in camera but has zero lidar hits for this geometry; do not treat this mount as safe low-obstacle detection. Instantaneous / noiseless sensors only; no navigation or FAST-LIO rerun in this test. Mesh physics limitations remain.

## Integration repository transition
New authoritative architecture and development plan: ../plan.md. Existing prototype and its evidence remain here until relocatable ROS-package migration is completed; no sensor/algorithm behavior changed by this workspace scaffold step.
