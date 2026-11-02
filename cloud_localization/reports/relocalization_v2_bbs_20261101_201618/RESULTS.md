# 重定位 V2 验证记录（2026-11-01）

## 实现

- V1 `grid_gicp` 保留原距离网格搜索与 GICP；V2 接入官方 CPU/GPU 3D-BBS。
- 官方仓库：https://github.com/KOKIAOKI/3d_bbs ，固定提交
  `41529a34a2fb9618b5ff560fb3c2363f1615666d`。本地候选集合扩展见
  `upstream_candidates.patch` 和 `vendor/3d_bbs/ROBOCON_INTEGRATION.md`。
- 三种后端共用候选 GICP、全点误差/重叠率与歧义检查、后台线程、轮速运动补偿和连续三帧确认。
- 新增 `GlobalSearchBackend`、工厂与 `globalRelocalize` 接口和独立 C++ 离线测量程序。
- BBS 搜索分辨率固定 0.20 m，不因搜索量变大而自动加粗；重力预对齐后
  roll/pitch 残差窗口 ±0.035 rad，yaw 全圈，XYZ 覆盖地图边界外约 1 m。

## 本机同一扫描的离线结果

CPU Intel Core Ultra 7 255HX，GPU RTX 5060 Laptop 8 GB，CUDA 12.9，Release。
CPU BBS 使用 2 线程；GPU 批量 10000；地图和扫描预先按 0.10 m 降采样。
目标为当前用户 launch 默认 `mid360_sim_2.pcd`；输入为 Gazebo 中采集的
`simulation_scan.pcd`（1929 个原始扫描点）。使用相同初始 roll/pitch=0、yaw=1.5708。
下面是 `comparison.json` 中实际记录的总计算时间，包含 BBS 地图准备和候选 GICP：

| 后端 | 第 1 次 ms | 第 2 次 ms | 第 3 次 ms |
|---|---:|---:|---:|
| V1 grid_gicp | 1221.69 | 1209.82 | 1210.01 |
| V2 bbs_cpu_gicp | 376.39 | 366.24 | 373.82 |
| V2 bbs_gpu_gicp | 191.03 | 40.57 | 39.40 |

GPU 第一次约 150.57 ms 用于 CUDA 上下文/地图初始化；后两次使用缓存。
GPU 搜索本身约 15 ms，GICP 约 25 ms。CPU BBS 搜索约 339～347 ms，
比 V1 纯搜索 143～150 ms 慢，但候选更集中，GICP 从约 1066 ms 降至约 27～35 ms，
所以端到端仍更快。不能把 GPU 稳态值当作第一次请求耗时。

三种结果均通过同一质量门限，最终全点 MSE 为 0.001478～0.001483 m²，
最近邻 RMSE 约 3.85 cm，0.20 m 内重叠率 100%。三种结果位置差在 8 mm 内，
yaw 差约 0.0012 rad 内。这些是对所选地图的一致性，未声明对 Gazebo 真值的绝对精度。
只有一个地图和一帧实测扫描、每种 3 次计算，不代表大场景/长时间行驶的统计性能，
也不能直接与论文的跨数据集平均 878 ms 比较。

复现全部 15 条结果：

```bash
source scripts/env.sh
python3 reports/relocalization_v2_bbs_20261101_201618/run_comparison.py
```

脚本也检查合成唯一场景三种算法均成功、两个相同目标相距 6 m 的对称地图均因歧义拒绝。
合成数据来自既有 `icp_initial_alignment_20261101_145322/check_initial_alignment.py` 的固定种子场景，
保留 PCD 以直接复现。退出非零会报断言错误，不只记录运行时间。

## ROS 与 Gazebo 行为验证

CPU/GPU CTest 均通过（`ctest.log`）：

- 倒装 180° 的雷达坐标系与非零 yaw 正确组合，XYZ/旋转误差小于 0.03 m/rad。
- 重复搜索复用地图，第二次 `map_build_ms=0`。
- 空扫描、非有限初始旋转、未知算法、缓存实例与算法名字不匹配被拒绝。
- 搜索超时被标记为 `timed_out` 且不接受结果。

三种后端分别运行完整编译后 ROS 节点，12 组场景全部通过
（`ros_grid_gicp.log`、`ros_bbs_cpu_gicp.log`、`ros_bbs_gpu_gicp.log`，每种四组）：

1. 局部初始化需要连续三帧；跟踪时 0.5 m 突变被原门限拒绝。
2. 低重叠扫描被拒绝。
3. 错误初值 XYZ=(3,-3,0)、yaw=1.2 rad 后自动恢复；定位后模拟无轮速提示的
   1.3 m 位移，再次丢失跟踪后自动恢复。
4. 对称场景全局候选被拒绝；RViz `/initialpose` 可重新初始化且保留高度/倾斜；
   `/relocalize` 请求能触发新搜索。

测试命令（逐个执行，避免与实际仿真使用同一 ROS domain）：

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=96
python3 reports/relocalization_v2_bbs_20261101_201618/check_backends_ros.py grid_gicp
python3 reports/relocalization_v2_bbs_20261101_201618/check_backends_ros.py bbs_cpu_gicp
python3 reports/relocalization_v2_bbs_20261101_201618/check_backends_ros.py bbs_gpu_gicp
```

完整 Gazebo 测试单独使用 domain 97、Gazebo master 11497、`gui:=false`，
错误初值 x=y=0、yaw=1.5708。GPU 首次全局计算 228.18 ms
（地图准备 174.96 / 搜索 19.46 / GICP 33.75 ms），随后通过三帧确认并稳定跟踪。
日志 `simulation_gpu.log`。该扫描与离线测量帧不是同一时刻，因此结果和耗时不完全相同。
测试后已关闭该组仿真进程。

## 代码与数据保留

修改前源码在 `before/`，校验在 `sha256_before.txt`。
`mid360_sim_1.pcd` 和 `mid360_sim_2.pcd` 的 SHA256 在修改前后完全一致；
launch 原地图选择及位姿默认值保持当前用户配置。
`changes.patch` 记录工作空间源码变化（官方核心修改另见 `upstream_candidates.patch`）。
`COLCON_IGNORE` 避免报告里的备份被识别为重复 ROS 包。

API、算法切换与参数优先级详见 `docs/relocalization_backends.md`。
