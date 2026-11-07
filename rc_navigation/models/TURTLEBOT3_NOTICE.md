# TurtleBot3 model resources

`turtlebot3_waffle/`, `turtlebot3_common/` and
`../urdf/turtlebot3_waffle.urdf` originate from the locally installed ROS 2
Humble `turtlebot3_gazebo` package (`/opt/ros/humble/share/turtlebot3_gazebo`).

Upstream: https://github.com/ROBOTIS-GIT/turtlebot3_simulations

Copyright ROBOTIS CO., LTD. The upstream package declares Apache-2.0.
The license text is included in `TURTLEBOT3_LICENSE`.

The SDF model files, model metadata and meshes are copied without changes.
The URDF's mesh URI prefix is changed from `package://turtlebot3_gazebo/`
to `package://rc_navigation/` so it can resolve the bundled resources.
