# 建图漂移与回环检查

`mapping.launch.py` 启动 `mode=map`。当前版本的建图链路为：

```text
FAST-LIO 局部扫描 + 同一时间的轮速/LIO 位姿
    → 关键帧 → 局部子图 GICP → 鲁棒局部几何/轮速/LIO 约束
    → Scan Context 历史候选 → 历史子图 GICP → 严格验证 + 连续关键帧确认
    → iSAM2 更新历史位姿 → 重建 optimized_map → /save_map
```

保存地图定位的 BBS 参数不控制这条建图链路。

## 为什么原来“有回环仍越建越歪”

2026-11-02 的用户运行日志实际接受了 98 个回环。旧版用单历史扫描匹配，
无空间候选约束、无修正幅度限制、无连续帧确认；MSE 门限 0.10 m²，
且用 0.8 m 作为内点距离统计“重叠率”。因此日志中的 100% 重叠并不表示几厘米级对齐。
0.08～0.10 m² 的匹配也被接受，可能把不准确的约束加入位姿图。
日志旧单位写成 m，实际应为 m²；原 `getFitnessScore` 距离参数还混用了平方距离。
仅凭截图无法逐个确认哪些约束为错误回环，但旧验收规则不足以防止这类破坏。

另一处确定的问题是原关键帧把原始 `/livox/lidar` 与回调时最新的轮速/LIO 位姿拼在一起，
没有按扫描时间配对。消息延迟、转弯和积压会造成点云与位姿不一致。
当前改为 `/fast_lio/cloud_registered_body`，该扫描与 FAST-LIO 位姿共用时间戳；
轮速使用时间插值，不跨越过大的测量间隔、不外推。该话题在当前零雷达/IMU外参配置下
与局部雷达坐标一致。更换实际传感器外参时也需同步检查后端使用的安装变换。

此外，旧版没有后端的局部扫描到子图约束，回环前主要依靠轮速与较弱的 LIO 相对因子。
现在增加局部几何配准约束，回环检测仍负责历史漂移修正。

## 如何运行与确认

```bash
cd /home/alice/rc_2027/cloud_localization
source scripts/env.sh
ros2 launch robocon_cloud_localization mapping.launch.py
```

重新启动后才使用新程序/参数。终端应出现：

- `Mapping input=/fast_lio/cloud_registered_body`：确认新的建图输入。
- `Mapping local GICP accepted/rejected`：局部几何约束是否通过。
- `Loop ... pending: 1/2 consistent keyframes`：历史回环等待第二个关键帧确认。
- `Accepted verified loop ... graph correction=...; rebuilding all keyframes`：
  回环已写入图，并打印对当前位姿实际产生的修正。
- `Rejected loop ... mse=... m^2 overlap=... correction=...`：未通过验收。
- `Mapping scan rejected: no time-aligned wheel/LIO poses`：缺少有效时间配对，
  应检查里程计时间戳/发布连续性，而非放宽回环误差。

回环需要足够的历史间隔和相似、重叠的几何；经过原来的区域也不保证任何一帧立即通过。
`/localization/optimized_map` 会随历史位姿更新，原始 FAST-LIO 地图不会被这条后端链路回写。
启动时地图坐标锚定第一帧雷达，不是 Gazebo 的 world。

## 建图参数

位置：`src/robocon_cloud_localization/config/localization.yaml`，修改后重启。

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `mapping_sync_max_gap` | 0.20 s | 允许用于位姿插值的最大里程计时间间隔 |
| `mapping_local_registration` | true | 是否添加局部扫描到子图几何约束 |
| `mapping_local_submap_frames` | 5 | 局部子图最多使用最近几帧 |
| `mapping_local_fitness_max` | 0.01 m² | 局部配准全点截断 MSE 上限 |
| `mapping_local_inlier_ratio_min` | 0.70 | 局部配准最小重叠比例 |
| `mapping_local_max_correction/max_rotation` | 0.30 m / 0.15 rad | 局部匹配相对轮速预测的最大修正 |
| `loop_search_radius` | 2.5 m | 图预测位置附近的历史候选搜索范围 |
| `loop_separation` | 12 帧 | 排除近期关键帧 |
| `loop_min_time` | 15 s | 与历史候选的最小时间间隔 |
| `loop_similarity` | 0.42 | Scan Context 差异上限，越小越严格 |
| `loop_candidates` | 3 | 最多验证几个候选，避免只尝试一个错误候选 |
| `loop_submap_frames` | 2 | 历史候选前后各加入几帧，排除近期帧 |
| `gicp_max_correspondence` | 0.8 m | 寻找 GICP 对应点的半径 |
| `loop_fitness_max` | 0.01 m² | 全点截断 MSE 上限，未匹配点计入误差 |
| `loop_inlier_distance` | 0.20 m | 判定内点的半径，供回环和局部配准质量评估 |
| `loop_inlier_ratio_min` | 0.70 | 历史回环最小内点比例 |
| `loop_max_correction/max_rotation` | 1.0 m / 0.35 rad | 相对图预测允许的最大回环修正 |
| `loop_consistent_frames` | 2 | 连续不同关键帧的确认数量 |
| `loop_consistency_translation/rotation` | 0.20 m / 0.10 rad | 轮速补偿后候选位姿允许的差异 |
| `loop_min_interval` | 3 s | 两次接受回环的最小间隔，避免反复加入同一处约束 |
| `loop_ambiguity_ratio` | 1.25 | 不同位姿有接近的匹配误差时拒绝回环，增大更严格 |

`keyframe_distance/yaw` 控制运动关键帧触发，`keyframe_min_interval` 控制频率，
`keyframe_voxel` 控制扫描降采样。已有 YAML 中的 `keyframe_max_*` 尚未被节点声明/使用，
添加这些键不会自行产生相应行为。

当前空间搜索和修正上限针对有连续轮速的场地仿真。漂移远超这些界限时会拒绝回环，
不能保证任意大的全局漂移恢复；重复/无特征平面也仍可能缺乏足够约束。

## 保存与后续定位

```bash
./scripts/save_map.sh
ros2 launch robocon_cloud_localization localization.launch.py \
  map_file:=/home/alice/rc_2027/cloud_localization/maps/fastlio_mid360_downward/map.pcd
```

`/save_map` 保存的是优化后的地图及关键帧。FAST-LIO YAML 中的
`pcd_save.pcd_save_path` 是另外一张前端累计地图，它不会包含后端回环修正。
若只需要优化地图，可在建图 launch 命令上加 `save_fastlio_pcd:=false`。
`map_directory:=/absolute/output/bundle` 可指定 `/save_map` 的输出目录。
既有聚合 PCD 不会因为更新程序而自动消除重影，应重新建图，或使用保留的原始关键帧另做离线优化。

验证结果、保留的运行日志和复现脚本见 `reports/mapping_consistency_20261102/`。
