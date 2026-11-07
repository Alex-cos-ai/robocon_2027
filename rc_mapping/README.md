# rc_mapping 独立建图工作空间

从 `Nazarite_vision_r2/src/rc_bringup/launch/mapping.launch.py` 提取的 MID360 建图链路，适用于 ROS 2 Humble / Ubuntu 22.04。

## 启动建图

本机编译完成后，在工作空间根目录执行：

```bash
cd ~/rc_2027/rc_mapping
source install/setup.bash
ros2 launch rc_mapping mapping.launch.py
```

其中 `source install/setup.bash` 只加载环境；`ros2 launch` 才启动建图。启动后会运行 MID360 驱动、FAST-LIO、点云转 LaserScan、slam_toolbox、固定机器人 TF 和 RViz。

关闭 RViz：

```bash
ros2 launch rc_mapping mapping.launch.py use_rviz:=false
```

用已有 rosbag 的 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）和 `/livox/imu` 建图：

```bash
ros2 launch rc_mapping mapping.launch.py use_livox:=false use_sim_time:=true
```

另开终端，在同一工作空间根目录加载环境后执行 `ros2 bag play /path/to/bag --clock`。录制包如果还包含 `/tf`、`/tf_static`、`/Odometry`、`/cloud_registered`、`/scan`、`/map`，应使用 `--topics /livox/lidar /livox/imu` 只回放原始传感器输入，避免与建图节点重复发布。

## 雷达与安装参数

雷达配置：`src/rc_mapping/config/MID360_driver.json`。沿用提取时正在使用的配置：雷达 `192.168.1.136`，电脑有线网卡 `192.168.1.50`。接入雷达的网卡需要具备该地址；更换地址时同步修改 JSON 中对应的 host IP 和雷达 IP。

保留原来的倒装雷达姿态、IMU 外参和 footprint 偏移，位于 `src/rc_mapping/urdf/mapping.urdf`。FAST-LIO 参数位于 `src/rc_mapping/config/fast_lio.yaml`。改变雷达安装方式时，需要同时校准 URDF 与 FAST-LIO 外参。

点云输入仍为 `/cloud_registered`，转为 `/scan` 后交给 slam_toolbox 生成 `/map`。高度范围、距离范围和分辨率沿用原建图配置。TF 为 `map -> odom -> base_link -> imu_link -> livox_frame`，其中 `map -> odom` 由 slam_toolbox 发布，`odom -> base_link` 由 FAST-LIO 发布。

本包不启动 Nav2 导航、STM32 串口、导航代理、机械臂、相机或关节状态节点。建图时需要通过已有的遥控方式移动机器人。

## 保存二维地图

保持建图运行，在另一个终端执行：

```bash
cd ~/rc_2027/rc_mapping
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f maps/map_1 --ros-args -p save_map_timeout:=10.0
```

结果为 `maps/map_1.yaml` 和 `maps/map_1.pgm`。这里只使用 Nav2 的地图保存工具，不运行导航栈。FAST-LIO 原有的退出时 PCD 保存功能也保留，输出到 `src/fast_lio/PCD/scans.pcd`；长时间建图会积累点云并增加内存占用。

## 编译与依赖

工作空间包含四个源码包：

```text
src/
  rc_mapping/                 # 独立 launch、配置、URDF 和 RViz
  livox_ros_driver2/           # MID360 驱动与自定义消息
  fast_lio/                   # 原工作空间的 FAST-LIO，编译建图节点
  pointcloud_to_laserscan/    # 原工作空间的点云转扫描实现
```

这些源码是独立副本，不通过软链接引用原工作空间。FAST-LIO 的滤波和堆叠检测节点不参与编译；保留原建图算法。驱动副本保留提取时已有的线程链接修正。

重新编译：

```bash
cd ~/rc_2027/rc_mapping
./build.sh
source install/setup.bash
```

`build.sh` 在清理过的子进程环境中仅加载系统 ROS 2 Humble，并顺序编译本目录的包，避免把原工作空间写入新安装环境。首次编译也可以直接使用这个脚本。默认单线程编译，以降低 FAST-LIO 编译时的内存峰值。

系统需要 ROS 2 Humble、colcon、PCL/Eigen/Boost、APR、Python 开发头文件，以及系统 ROS 包 `slam_toolbox`、`rviz2`、`robot_state_publisher`、`nav2_map_server` 和各源码包声明的依赖。可通过 `rosdep install --from-paths src --ignore-src --rosdistro humble -r -y` 安装系统依赖。

Livox-SDK2 是额外的系统依赖，本机已经安装：`/usr/local/include/livox_lidar_api.h` 和 `/usr/local/lib/liblivox_lidar_sdk_shared.so`。移到另一台电脑时需要先安装 SDK2；本目录不携带其二进制库。

本机的 MVS 相机 SDK 另带旧版 libusb，可能让 PCL 启动时报 `undefined symbol: libusb_set_option`。启动文件为雷达驱动和 FAST-LIO 子进程优先选择系统库目录，避免该冲突；不修改系统库和终端环境。

## 可覆盖的启动参数

执行 `ros2 launch rc_mapping mapping.launch.py --show-args` 查看参数。可覆盖 `use_rviz`、`use_livox`、`use_sim_time`、`livox_config`、`fast_lio_config`、`slam_config`、`rviz_config` 和 `map_file_path`。`map_file_path` 只控制原 FAST-LIO `/map_save` 服务的输出位置，其父目录需存在；该服务还要求启用 `publish.map_en` 才有累积地图数据。二维地图使用前面的 `map_saver_cli` 保存。

## 来源

提取时原仓库版本为 `679257eb3cabfe3a1fb9168985714dcc61f734b1`，Livox 驱动版本为 `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`，并保留了当前驱动副本及 MID360 配置的本地修改。各第三方包原有的许可证文件保留在其目录中。原工作空间和 `rc_2027` 中其他项目的已有修改未作编辑。

## 本机验证（2026-11-06）

- 只依赖系统 ROS 2 Humble 的干净环境中，四个包全部编译成功；`rosdep check` 确认系统依赖齐全。
- 四个包均从本工作空间解析；运行环境和动态库不引用原工作空间。
- 合成 MID360 自定义点云与 IMU 数据跑通 FAST-LIO → `/cloud_registered` → `/scan` → `/map`，产生 120 × 120、分辨率 0.05 m 的地图，其中 4782 个已知栅格、674 个占用栅格。
- `map_saver_cli` 成功保存测试地图的 YAML 与 PGM；FAST-LIO 退出保存 PCD、各建图节点正常退出。
- RViz 成功加载本包配置与显示插件。
- 雷达驱动可加载二进制与 JSON 配置。测试时电脑只有 Wi-Fi 地址，缺少 `192.168.1.50` 有线地址，实际驱动报告 `bind failed`；因此以上是合成数据验证，真实雷达采集与实地建图仍需接入 MID360 后验证。
