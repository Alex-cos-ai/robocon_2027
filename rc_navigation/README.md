# rc_navigation

ROS 2 Humble package for the Robocon 2027 Gazebo Classic field and Nav2 navigation experiments. It uses a TurtleBot3 Waffle differential-drive model with a planar 2-D LaserScan on `/scan` and wheel odometry/TF on `/odom`; AMCL is the default localizer. No MID360, PointCloud2, IMU, or point-cloud localization nodes are launched.

## Build and run

```bash
cd /home/alice/rc_2027/rc_navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rc_navigation navigation.launch.py
```

The default launch starts Gazebo, the robot at `(-5.15, 5.15, 0)`, map_server, AMCL, Nav2 and RViz. The default map is a conservative 5 cm occupancy grid projected from the field's static geometry to provide an initial navigation map. It treats raised arena platforms and game pieces as blocked regions; regenerate a scan-derived map for the exact operating height/terrain before using the map as a final competition map.

Use the RViz **2D Goal Pose** tool to send a navigation goal. Initial pose and Gazebo spawn pose default to the same coordinates; change both through the launch args when testing another start pose:

```bash
ros2 launch rc_navigation navigation.launch.py initial_pose_x:=-4.9 initial_pose_y:=4.9 initial_pose_yaw:=0.0
```

Useful launch options:

```bash
ros2 launch rc_navigation navigation.launch.py gui:=false start_rviz:=false
ros2 launch rc_navigation navigation.launch.py start_nav2:=false  # Gazebo and robot only
ros2 launch rc_navigation navigation.launch.py world:=/path/to/other.world map:=/path/to/map.yaml
```

## Build a scan-derived map

To create a map with the actual 2-D scan and wheel odometry, use online SLAM mode, teleoperate the robot through the areas you need, then save the occupancy grid:

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
- `odom -> base_footprint`: Gazebo differential-drive plugin from wheel odometry.
- `base_footprint -> base_link -> base_scan`: robot_state_publisher.
- `/scan` (`sensor_msgs/msg/LaserScan`), 720 rays, 20 Hz, 0.12–12 m.
- `/odom` (`nav_msgs/msg/Odometry`), 30 Hz.
- `/cmd_vel` (`geometry_msgs/msg/Twist`), Nav2 output to Gazebo.

`config/nav2_params.yaml` includes AMCL, mapping-mode slam_toolbox, costmaps, NavFn, behavior tree navigation, velocity smoothing, recovery behaviors, and the MPPI controller. The MPPI model timestep is 0.05 s and the controller runs at 20 Hz. `visualize` is off for normal timing; set it to `true` under `controller_server.FollowPath` to inspect sampled trajectories during tuning.
