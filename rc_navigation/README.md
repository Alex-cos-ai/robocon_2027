# rc_navigation

ROS 2 Humble package for the Robocon 2027 Gazebo Classic field and Nav2 navigation experiments. It spawns the stock TurtleBot3 Waffle Gazebo model, including the original chassis, wheels, LDS laser, IMU and camera. AMCL uses the planar `/scan` and differential-drive-plugin `/odom`; no MID360 or point-cloud localization is started. The Gazebo plugin defaults to world ground-truth odometry (`odometry_source=1`), not encoder integration.

## Build and run

```bash
cd /home/alice/rc_2027/rc_navigation
source install/setup.bash
colcon build --packages-select rc_navigation
source install/setup.bash
ros2 launch rc_navigation navigation.launch.py
```

Gazebo opens with a fixed overview from behind the default spawn corner toward the field center. It shows the full arena and the robot's route into the playing area.

The default launch starts Gazebo, the robot at `(-5.15, 5.15, 0)`, map_server, AMCL, Nav2 and RViz. The current default map is `maps/map_1.yaml`; its coordinate extent is approximately `x=[-5.44, 4.91]`, `y=[-2.07, 5.18]` metres. Navigation goals must be inside the free part of this map and use its `map` coordinates. As with any localization map, regenerate a scan-derived map for the exact operating height/terrain before using a geometric projection as a final competition map.

Use the RViz **2D Goal Pose** tool to send a navigation goal. The current parameter file leaves AMCL automatic initialization disabled; use **2D Pose Estimate** first. The following arguments change the Gazebo spawn pose and rewrite AMCL initial coordinates when the `initial_pose` parameter block is enabled:

```bash
ros2 launch rc_navigation navigation.launch.py initial_pose_x:=-4.9 initial_pose_y:=4.9 initial_pose_yaw:=0.0
```

For repeatable point-to-point testing, send a single goal in map coordinates:

```bash
ros2 launch rc_navigation navigate_to_pose.launch.py x:=-3.35 y:=0.85 yaw:=0.0
```

First start `navigation.launch.py` (or `navigation.launch.py slam:=true` while mapping), and set the initial pose in RViz if needed. Then start this separate goal launch. Coordinates and `yaw` are relative to the active `map` frame; yaw is in radians (for example, `1.5708` faces 90 degrees counterclockwise). The goal node waits for Nav2 and the `map -> base_footprint` transform, reports progress, and cancels on `timeout` or Ctrl+C. Set a longer timeout with `timeout:=240`. If Nav2 is running in SLAM mode, the current map frame is still named `map` and can be used for goals.

Useful launch options:

```bash
ros2 launch rc_navigation navigation.launch.py gui:=false start_rviz:=false
ros2 launch rc_navigation navigation.launch.py start_nav2:=false  # Gazebo and robot only
ros2 launch rc_navigation navigation.launch.py world:=/path/to/other.world map:=/path/to/map.yaml
```

## Build a scan-derived map

To create a map with the actual 2-D scan and `/odom`, use online SLAM mode, teleoperate the robot through the areas you need, then save the occupancy grid:

```bash
ros2 launch rc_navigation navigation.launch.py slam:=true
# In another terminal after sourcing the same workspace:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
ros2 run nav2_map_server map_saver_cli -f /home/alice/rc_2027/rc_navigation/maps/robocon_track
```

Stop the running launch, then relaunch with the saved map (AMCL is the default):

```bash
ros2 launch rc_navigation navigation.launch.py map:=/home/alice/rc_2027/rc_navigation/maps/robocon_track.yaml
```

## Frames and topics

- `map -> odom`: AMCL, using the saved occupancy map and `/scan`.
- `odom -> base_footprint`: Gazebo differential-drive plugin; world ground truth by default. Encoder integration requires explicitly setting `<odometry_source>0</odometry_source>` in the plugin configuration.
- `base_footprint -> base_link -> base_scan`: robot_state_publisher.
- `/scan` (`sensor_msgs/msg/LaserScan`), stock LDS: 360 rays, 5 Hz, 0.12–3.5 m.
- `/imu` and `/camera/image_raw`, from the stock Gazebo sensors; not used as AMCL observations.
- `/odom` (`nav_msgs/msg/Odometry`), 30 Hz.
- `/cmd_vel` (`geometry_msgs/msg/Twist`), Nav2 output to Gazebo.

`config/nav2_params.yaml` includes AMCL, mapping-mode slam_toolbox, costmaps, NavFn, behavior tree navigation, velocity smoothing, recovery behaviors, and the MPPI controller. The MPPI model timestep is 0.05 s and the controller runs at 20 Hz. `visualize` is off for normal timing; set it to `true` under `controller_server.FollowPath` to inspect sampled trajectories during tuning.

## Stock Waffle model

Gazebo loads `models/turtlebot3_waffle/model.sdf` directly, as in the original TurtleBot3 launcher. The model and `models/turtlebot3_common` meshes are copied from the installed ROS 2 Humble `turtlebot3_gazebo` package. The SDF is unmodified: sensor poses, wheel friction, inertias and drive settings are the original values. `GAZEBO_MODEL_PATH` includes the local `models` directory so the chassis and wheel meshes resolve without downloading models.

The `slam` argument changes localization nodes, not robot physics. The stock drive plugin retains its last velocity target, so stopping command publication is not a stop command: publish a zero `Twist` when stopping teleoperation.

`urdf/turtlebot3_waffle.urdf` provides the matching official TF description, with only mesh package paths changed to this package. The LDS sensor pose in the stock SDF is `(-0.064, 0, 0.121)` in its `base_scan` link (the stock SDF leaves that link pose at the model origin). The stock URDF defines `base_link -> base_scan` as `(-0.064, 0, 0.122)` and `base_footprint -> base_link` as `(0, 0, 0.010)`. These original files have a small pose discrepancy; they have not been independently adjusted. The launch spawn height is the stock `0.01 m`.

`robot_sdf:=...` selects a Gazebo model and `robot_xacro:=...` selects its matching TF description. `urdf/robot_2d.xacro` is retained as a compatibility wrapper around the stock URDF and no longer injects custom sensors or drive plugins.
