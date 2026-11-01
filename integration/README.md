# XLeRobot JT16 integration

Initial **static sensor baseline**, not a validated navigation system. See plan.md for results and outstanding work.

Run `scripts/run_sensor_smoke.sh` in a ROS 2 Jazzy environment with Gazebo Harmonic installed. Do not concurrently run another copy in the same partition. Models generated from upstream XLeRobot; current simulation mount is ground-relative (-0.03, 0, 0.36)m with zero roll/pitch/yaw. Shell/inertia and uniform channel elevations remain provisional. The cloud adapter assembles ten snapshots into a rolling scan, but this is not measured JT16 timing/noise. Upstream zero-mass decorative inertias are regularized only in derived URDF. Static smoke does not validate wheel dynamics, hardware extrinsics or collision safety. The old 0.676350 rad bracket is historical, not the current simulation pose.

Build FAST-LIO using system Python 3.12 with colcon. Missing pcl_ros was downloaded into deps/root; no privileged system install succeeded. Build artifacts live in build/, install/, log/. Evaluation artifacts live in results/. Do not add generated artifacts to Git.

Sensor frame conventions and timestamp synchronization must pass motion tests before navigation. A static odometry sample has nonzero displacement; precision is not accepted.

Mount updated on 2026-10-30; previous static sensor / FAST-LIO evidence used the old 0.55m level mount and is historical, not validation of this new pose.

After 2026-10-30 move, vendor roots are ../external/runtime/ and ../external/reference/. Old build/install/log caches are under archive/pre-move; rebuild before running FAST-LIO.
