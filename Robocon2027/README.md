感谢 [https://github.com/jimi4-lei/](https://github.com/jimi4-lei/gazebo_models)开源
修改自 [https://github.com/jimi4-lei/gazebo_models](https://github.com/jimi4-lei/gazebo_models)，修改了正确的天空块的摆放方式。

以下方式可以启动：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon2027_description --symlink-install
source install/setup.bash
ros2 launch robocon2027_description robocon2027_world.launch.py
