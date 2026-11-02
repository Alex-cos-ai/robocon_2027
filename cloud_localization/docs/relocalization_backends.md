# 重定位 V2：3D-BBS 与 GICP 的切换接口

全局搜索采用官方 [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs)，固定版本
`41529a34a2fb9618b5ff560fb3c2363f1615666d`，MIT 许可。
这属于点云全局配准：多分辨率稀疏哈希体素 + 旋转/平移联合分支定界搜索，
GPU 版本批量计算节点分数。它不做图优化，也不使用 Gazebo 真值。

## 1. 直接切换测试

在工作空间执行，三次实验分别启动，退出上一组 launch 后再启动下一组：

```bash
cd /home/alice/rc_2027/cloud_localization
source scripts/env.sh

# V1：原来的距离网格搜索 + GICP
ros2 launch robocon_cloud_localization localization.launch.py relocalization_backend:=grid_gicp

# V2：官方 CPU BnB + GICP
ros2 launch robocon_cloud_localization localization.launch.py relocalization_backend:=bbs_cpu_gicp

# V2：官方 CUDA 批量 BnB + GICP（当前默认）
ros2 launch robocon_cloud_localization localization.launch.py relocalization_backend:=bbs_gpu_gicp
```

三种方案共用地图、输入扫描、GICP 精配准、重叠率/误差/歧义检查和连续三帧确认。
日常局部跟踪始终使用原有 GICP。只有连续失败或显式请求时才调用所选全局搜索。
测试时固定相同的 `map_file`、Gazebo 出生位置和初始定位位姿。例如在上述任一命令后加
`initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=1.5708`。
当前位置已能局部收敛时，应使用错误初值重新启动，确保确实测到全局搜索。

当前外层 launch 的地图默认值是 `mid360_sim_2.pcd`；切换地图可以显式传入
`map_file:=/home/alice/rc_2027/cloud_localization/src/robocon_cloud_localization/maps/mid360_sim_1.pcd`。

`relocalization_backend` 是 launch 参数，写入节点 ROS 参数
`localization_global_backend`，因此 **launch 参数覆盖 YAML 中同名 ROS 参数**。
用以上命令选择算法最直接。只启动节点时，使用 `-p localization_global_backend:=...`。

可核对实际选择，并主动请求搜索：

```bash
ros2 param get /scan_context_gicp_isam2 localization_global_backend
ros2 service call /relocalize std_srvs/srv/Trigger '{}'
```

服务只确认已收到请求；后续扫描触发搜索。
`Global backend=...` 输出计算结果，`Global relocalization candidate` 之后还需
`GICP initialized` 才完成新扫描确认。局部对齐先完成时，会作废后台的旧搜索结果。

## 2. C++ 算法接口

声明：`src/robocon_cloud_localization/src/global_relocalization.hpp`。

```cpp
robocon::GlobalOptions options;
options.backend = "bbs_gpu_gicp";  // grid_gicp / bbs_cpu_gicp / bbs_gpu_gicp
auto backend = robocon::makeGlobalSearchBackend(map, options);  // 每张地图创建一次
auto result = robocon::globalRelocalize(scan, map, guess, options, backend);
if (result.found) {
  Eigen::Matrix4f T_map_lidar = result.pose;
}
```

- `map`：目标 PCD 地图，`pcl::PointCloud<pcl::PointXYZI>::ConstPtr`。
- `scan`：当前雷达局部坐标系扫描，同一类型。不要传已经变换到 map 的点云。
- `guess`：`Eigen::Matrix4f`，初始 `T_map_lidar`。BBS 不围绕它的 XYZ/yaw 做局部窗口搜索；
  只使用其 roll/pitch 作为重力/安装方向预对齐。平移搜索覆盖地图边界外约 1 m，yaw 覆盖整圈。
  BBS 额外搜索预对齐附近的 roll/pitch；GICP 最后精修完整 6DoF。
- `result.pose`：`p_map = T_map_lidar * p_lidar`，只在 `found=true` 时有效。
- `result` 另含 `backend`、`timed_out`、`detail`、`fitness`（全点截断距离 MSE，m²）、
  `overlap`（0～1）、`coarse_score`（BBS 体素命中比例，0～1）及四个耗时字段。

扩展第三种搜索器时，实现 `GlobalSearchBackend::search(scan, guess) -> GlobalSeeds`，
返回候选 `T_map_lidar`，并在工厂注册名字。共同的 `refineGlobalCandidates` 继续承担 GICP 验证。
后端实例绑定创建时的地图与搜索配置，地图需保持不变；换地图或调搜索参数后重新创建。
一个实例每次只允许一个线程搜索。ROS 节点在单个后台任务中调用，缓存地图体素层级。
未知算法、编译时未启用 GPU 或 GPU 不可用都会明确报错，不静默换算法。

## 3. 参数位置与含义

配置文件：`src/robocon_cloud_localization/config/localization.yaml`。
调整 BBS 参数后重启节点，避免继续使用已缓存的配置/地图。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `localization_bbs_resolution` | 0.20 m | 最细体素边长；固定分辨率，不随搜索量自动加粗 |
| `localization_bbs_levels` | 5 | 最大层级，层级 0～5，每层边长翻倍 |
| `localization_bbs_scan_voxel` | 0.25 m | BBS 查询点云降采样；不改变发布地图分辨率 |
| `localization_bbs_roll_pitch_window` | 0.035 rad | 重力预对齐后 roll/pitch 搜索半宽 |
| `localization_bbs_score_threshold` | 0.60 | BBS 占用体素命中比例下限，不是 GICP fitness |
| `localization_bbs_timeout_ms` | 3000 ms | 搜索时限；不含地图初始化/GICP，检查间隔会有超时余量 |
| `localization_bbs_cpu_threads` | 2 | CPU 评分线程数 |
| `localization_bbs_gpu_batch_size` | 10000 | GPU 节点评分批量 |

原 `localization_global_voxel/xy_step/z_step/yaw_step/refine_candidates` 仅控制 V1 网格搜索。
共同的 GICP 验证仍使用 `localization_initial_fitness_max`、
`localization_initial_inlier_distance/ratio_min` 和 `localization_global_ambiguity_ratio`。

官方只输出一个最优位姿。本地对 CPU/GPU 核心做了小范围扩展：保留分数在最佳值 95% 以上、
最多 8 个位置/yaw 不同的叶节点，送入共同的 GICP 歧义检查。
空间去重阈值为 0.75 m / 0.35 rad。修改和许可记录见 `vendor/3d_bbs/ROBOCON_INTEGRATION.md`。
这不是穷尽所有歧义位置的保证。重复结构仍可能需要 RViz 初值；roll/pitch 偏差超过窗口也可能失败。
超时结果直接拒绝，不把未完成搜索当作定位成功。

## 4. 离线公平比较

新增独立 C++ 可执行程序，直接调用上述同一套代码，无需启动 Gazebo：

```bash
ros2 run robocon_cloud_localization relocalization_benchmark \
  /absolute/map.pcd /absolute/scan_in_lidar_frame.pcd bbs_gpu_gicp 3 0 0 0
```

位置参数：`MAP SCAN BACKEND [REPEATS [ROLL PITCH YAW]]`，角度为弧度。
地图/扫描先按 0.10 m 降采样，与当前 ROS 节点输入条件一致。
在同一个进程里重复，第一轮含 BBS 地图建立/GPU 上下文初始化，后续使用缓存。
每行 `RESULT {...}` 为 JSON：`map_build_ms`、`search_ms`、`refine_ms`、`total_ms`。
总耗时含地图建立与 GICP，不含磁盘加载、ROS 等待和新扫描确认。
退出码：0 全部成功；1 有匹配被拒绝；2 参数/运行错误。
不要将单独的 BBS 搜索耗时与另一方案的总耗时比较。

实测扫描、三种算法结果、ROS 回归日志和测试说明见
`reports/relocalization_v2_bbs_20261101_201618/RESULTS.md`。

## 5. 构建

当前机器已经编译并验证 CPU/GPU。本机 CUDA 12.9、RTX 5060 Laptop，架构 120：

```bash
source scripts/env.sh
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-select robocon_cloud_localization --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DROBOCON_BBS_CUDA=ON -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=120
ctest --test-dir build/robocon_cloud_localization --output-on-failure
```

其他 GPU 使用自己的 CUDA 路径和架构。无 CUDA 可传 `-DROBOCON_BBS_CUDA=OFF`，
启动时选择 `bbs_cpu_gicp`。依赖 Eigen >= 3.4、OpenMP、PCL；上游代码已放入 `vendor/3d_bbs`，
不向系统目录安装。换机复制源码时应保留该目录及其本地补丁，直接重新 clone 会丢失候选接口扩展。
