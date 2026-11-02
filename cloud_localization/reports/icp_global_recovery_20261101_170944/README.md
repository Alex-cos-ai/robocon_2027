# Robocon 2027 FAST-LIO2 simulation workspace

This workspace includes the Robocon 2027 Gazebo Classic field, a TurtleBot3 Waffle differential-drive chassis, a virtual MID360-style ray sensor, the ROS 2 port of FAST-LIO2, a Scan Context + GICP loop detector, and a GTSAM/iSAM2 incremental pose graph. FAST-LIO2 consumes the simulated scans and IMU; the pose graph also uses wheel odometry to anchor motion where the field geometry is degenerate. `/simulation/ground_truth` is published only for evaluation and is never consumed by the mapping or localization nodes. The Waffle URDF and meshes come from the ROS 2 `turtlebot3_description` package.

The simulation loads `worlds/robocon_track.world`; `robocon_ground` spans `x/y = [-5.5, 5.5] m`. The launch sets `GAZEBO_MODEL_PATH` to this package's installed `models` directory, so the field and tower models resolve locally. The Waffle uses its two driven wheels and consumes `/cmd_vel` while publishing `/odom`. The 2-D laser is removed. The virtual MID360 ray and IMU sensors are mounted at `(-0.10, 0, 0.45) m` relative to `base_link`, with a 180-degree roll. Its Gazebo ray sensor uses 360 horizontal by 32 vertical samples at 10 Hz, a 70 m range, and publishes standard `x/y/z/intensity` PointCloud2 to `/livox/lidar`; this approximates the old simulation, not Livox firmware or its return pattern. The localization default spawn `(-5.15, 5.15)` is at the center of the upper-left start marking with clearance from the field edge.

The source tree contains the Waffle-based simulation description and the ROS 2 FAST-LIO2 fork in `src/`, plus GTSAM 4.2 in `vendor/`. The application packages and forked sources build together in one colcon workspace. ROS 2 Humble, Gazebo Classic, TurtleBot3 description assets, PCL, and compiler/system development libraries come from the system installation; no package is loaded from the neighboring Robocon workspace. `scripts/env.sh` clears older overlays before sourcing `/opt/ros/humble` and this workspace's local installation.

## Build

```bash
./scripts/build.sh
```

This builds and installs the pinned GTSAM source locally, then runs `colcon build` serially. The initial GTSAM compile can take several minutes.

## Run the simulation and drive the robot

```bash
source install/setup.bash
ros2 launch robocon_cloud_localization simulation.launch.py
```

Gazebo and RViz open by default; add `gui:=false` for a headless run. The
workspace setup hook also exports the local GTSAM library path. Use
`source ./scripts/env.sh` when you want a clean shell that clears any older ROS
or Gazebo overlays before loading this workspace.

Use a second terminal with the same workspace setup for keyboard control. The bundled driver continuously sends commands while it runs; press **W/S** to drive forward/back, **A/D** to rotate, **Q/E** to drive in arcs, and **Space** to stop. The terminal running the driver must have keyboard focus:

```bash
cd ~/rc_2027/cloud_localization
source install/setup.bash
./scripts/drive.sh --pattern manual
```

The workspace does not force `ROS_LOCALHOST_ONLY`, so the simulator can also
discover keyboard nodes started from a terminal that only sources `/opt/ros/humble`.
Both terminals still need the same `ROS_DOMAIN_ID`; matching domain IDs alone
does not help if one terminal restricts ROS discovery to localhost.

The standard `teleop_twist_keyboard` node uses different keys: `i`/`,` forward/back and `j`/`l` rotate. The Waffle is differential drive and cannot strafe. It publishes on key events, so repeated key presses extend motion; its terminal also needs focus. Start it in a second sourced terminal with an explicit topic remap:

```bash
cd ~/rc_2027/cloud_localization
source install/setup.bash
unset ROS_LOCALHOST_ONLY
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

RViz opens with the loaded target map and live MID360 scan enabled. The FAST-LIO2 registered scan is available in Displays but starts hidden because the field geometry can make the raw FAST-LIO trajectory drift.

In mapping mode, RViz's live robot TF is propagated from the last optimized keyframe using `/odom` increments. This prevents the displayed chassis from following FAST-LIO2's unbounded drift while the simulated robot is stopped; FAST-LIO2 relative motion remains a robust secondary pose-graph factor.

## Map the field

```bash
ros2 launch robocon_cloud_localization mapping.launch.py initial_x:=-3.1 initial_y:=1.3
./scripts/drive.sh --pattern perimeter --side 0.8 --speed 0.20 --yaw-rate 0.8 --loops 3
# Or keyboard: ./scripts/drive.sh --pattern manual
./scripts/save_map.sh
```

The `mapping.launch.py` entry point in `src/robocon_cloud_localization/launch/` starts the shared `simulation.launch.py` with `mode:=map`. The tested route starts at `(-3.1, 1.3)` and repeats a 0.8 m square so the loop detector sees revisited scans. For a different route, set `initial_x` and `initial_y` to a clear point on the field. Keyframes retain each local MID360 ray scan; Scan Context proposes revisits, GICP must pass fitness and inlier-overlap gates, and accepted loops update iSAM2. `/localization/optimized_map` and `/localization/optimized_path` show optimized outputs. `/save_map` writes `maps/fastlio_mid360_downward/map.pcd`, one local-cloud PCD per optimized keyframe, optimized poses and `map_info.txt`.

FAST-LIO also saves its frontend point cloud independently of the pose-graph
map above. In `src/fast_lio_ros2/config/mid360_sim.yaml`, enable
`pcd_save.pcd_save_en` and set `pcd_save.pcd_save_path` to the output PCD.
`pcd_save.interval: -1` saves on normal shutdown (Ctrl-C); a positive value
saves every N processed scans and again on shutdown. For example, `100`
checkpoints approximately every 10 seconds at 10 Hz. Each checkpoint replaces
the file with the accumulated cloud in `lidar_init`, without pose-graph loop
corrections. This works with `publish.map_en: false`. Restart FAST-LIO after
editing these parameters. To save immediately while it is running:

```bash
ros2 service call /fast_lio/map_save std_srvs/srv/Trigger '{}'
```

Use `/save_map` (or `./scripts/save_map.sh`) for the separate optimized
pose-graph map in `maps/fastlio_mid360_downward/map.pcd`.

## Relocalize against a saved PCD

```bash
ros2 launch robocon_cloud_localization localization.launch.py
```

The `localization.launch.py` entry point starts the shared simulation in `mode:=localize`. Choose the target with `map_file:=/absolute/path/to/map.pcd`; check the effective defaults with `ros2 launch robocon_cloud_localization localization.launch.py --show-args`. FAST-LIO's automatic PCD saving is disabled during localization so the target cannot be overwritten on shutdown.

`initial_x`, `initial_y` and `initial_yaw` set the Gazebo spawn. `initial_pose_x/y/z` and `initial_pose_roll/pitch/yaw` set the initial **lidar** pose in the PCD map, in metres and radians. These are independent coordinate systems. A FAST-LIO map is anchored near the original mapping start, so world coordinates cannot simply be copied into the initial map pose. Set these arguments in the outer `localization.launch.py` or on the command line; values passed by that launch override defaults in `simulation.launch.py`. Restart the launch after editing them.

Localization now has two stages, configured in `src/robocon_cloud_localization/config/localization.yaml`:

- Initialization allows up to 1.0 m / 0.35 rad correction. It requires fitness <= 0.01 m^2, at least 60% of all scan points within 0.20 m of the target, and 3 consecutive candidates agreeing within 0.10 m / 0.05 rad after wheel-motion compensation. The validated initial alignment is applied in full once.
- Tracking restores the 0.35 m / 0.25 rad correction limits and blends accepted GICP corrections using `localization_icp_weight`. Failure during tracking keeps wheel prediction; it does not automatically enable the wider initialization limits again.

`GICP initializing/tracking candidate` logs include `map_delta_xyz` in **map axes**, relative rotation in degrees, and the active correction limits. `GICP initialized` prints the accepted full map pose, with rotation in **radians**, for reuse as an initial guess. PCL fitness is mean squared nearest-neighbor distance (m^2), not metres. A low score alone does not guarantee the correct field location on repetitive geometry; the initial guess must still be near the intended location.

For example, to supply an initial height offset without changing your other launch defaults:

```bash
ros2 launch robocon_cloud_localization localization.launch.py initial_pose_z:=0.615
```

In the simulation check recorded under `reports/icp_initial_alignment_20261101_145322`, the configured `mid360_sim_2.pcd` required approximately `[-0.095, +0.071, +0.615]` m relative to the existing initial guess. The previous 0.35 m tracking gate rejected that height correction indefinitely. With the separate initialization gate, all scan points were within 0.20 m of the map and nearest-neighbor RMSE was about 0.04 m before and after a short forward drive. This is a local registration check, not a global map calibration.

The relocalizer publishes `/localization/pose`, `map -> odom` and `map -> lidar_init`. The launch also publishes configurable static `world -> map` calibration; replace it with the measured field-to-PCD transform if Gazebo world geometry must align with the PCD.

After measuring the calibration transform, pass its translation in metres and roll/pitch/yaw in radians, for example:

```bash
ros2 launch robocon_cloud_localization localization.launch.py \
  world_to_map_x:=1.20 world_to_map_y:=-0.40 world_to_map_yaw:=0.0873
```

This TF only relates the `world` and `map` frames; it does not calculate the calibration values or make a mismatched PCD align automatically.

## Validation

Build, start `localization.launch.py`, then run the verifier in a second sourced terminal:

```bash
cd /home/alice/rc_2027/cloud_localization
source install/setup.bash
ros2 run robocon_cloud_localization verify_sim.py --duration 18 --localize
```

The verifier requires simulated lidar and IMU, wheel and FAST-LIO2 odometry, robot motion, at least one accepted GICP update, and no Waffle footprint crossing the field boundary. It reports scan counts, accepted ICP fitness, localization/wheel travel and ground-truth motion. It drives a small forward-and-turn square that stays inside the upper-left part of the field.

FAST-LIO2 odometry is retained as a secondary output; localization propagates the Waffle wheel odometry and applies accepted GICP corrections against the loaded map. Use a second sourced terminal for driving or inspecting topics.

## Input/output topics

- `/livox/lidar` (`sensor_msgs/PointCloud2` with `x/y/z/intensity`): local Gazebo physics rays, consumed through FAST-LIO2's standard PointCloud2 path.
- `/livox/imu`: simulated gyro and accelerometer in `mid360_link`.
- `/fast_lio/position`, `/fast_lio/cloud_registered`, `/fast_lio/cloud_registered_body`: FAST-LIO2 outputs.
- `/localization/optimized_map`, `/localization/optimized_path`, `/localization/pose`: GICP loop-verified/iSAM2 map products and localizer pose.
- `/simulation/ground_truth`: evaluation-only simulator pose; deliberately unused by estimation.
- `/save_map` (`std_srvs/Trigger`): save the optimized keyframes and map.
