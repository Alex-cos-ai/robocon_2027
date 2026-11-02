# 建图一致性修正与验证（2026-11-02）

## 用户这次日志提供的证据

保留原日志 `user_mapping_before.log`：233 个关键帧，98 个回环被接受。
存在 MSE 0.079～0.100 m² 的已接受匹配；旧内点半径为 0.8 m，
“100% overlap”不能证明墙面已精确重合。旧日志单位写成 m，实际为 m²。
无法仅凭截图和无原始扫描的日志逐个判定哪个回环错误；可确定旧版验收过宽，
且源码缺少空间/创新量/连续帧检查，单个错误候选就可能影响整张位姿图。
源码还把原始扫描和最新里程计直接拼接，没有按扫描时间配对。

## 修改

1. 建图改用 `/fast_lio/cloud_registered_body`，匹配相同时间戳的 LIO 与插值轮速位姿；
   缺少有效时间包围区间时等待/拒绝，不外推。建图实时 TF 也按同一时间取轮速。
2. 增加局部扫描到最近关键帧子图的 GICP 鲁棒约束，在历史回环前约束局部漂移。
   对应点搜索半径与验收半径分开；全点截断 MSE 统计未匹配点。
3. 历史回环使用空间/时间筛选的多个 Scan Context 候选及历史子图。
   默认 MSE <=0.01 m²、0.20 m 内重叠率 >=70%、修正 <=1 m /0.35 rad。
   不同位置有相近误差时拒绝；还需两个连续不同关键帧验证一致，才添加图约束。
4. 回环后额外迭代 iSAM2，原有发布/导出路径按更新后的每个历史位姿重建点云。
   新日志打印匹配误差、重叠、创新量和实际图修正，避免用“回环计数增加”替代质量验证。
5. 新 launch 参数 `save_fastlio_pcd` 可关闭独立前端 PCD 保存，
   `map_directory` 可选择优化地图输出目录。原默认行为/用户地图内容保留。

源码关键点：`loop_closure_pgo.cpp`、`mapping_registration.{hpp,cpp}`、
`stamped_pose_buffer.hpp`、`config/localization.yaml`。
保存地图定位的 BBS/GICP 后端仍保留；本次改变的是建图链路。

## 合成 ROS 集成测试

`check_mapping.py` 运行真实编译后的节点，21 个往返扫描，场景包含地面、两面垂直墙和不对称物体。
已知真实轨迹人为叠加逐帧累计的 XYZ/yaw 里程计误差。
故意在扫描到达前先发送更新、更偏的里程计，检验不会再使用最新消息错误贴图。
同时检查 `/save_map` 实际导出的地图、关键帧和元数据。

| 当前代码中的对照设置 | 返回起点误差 | 点云到真实场景 RMSE | 局部因子 / 回环 |
|---|---:|---:|---:|
| 仅里程计，关闭局部配准和回环 | 0.39395 m | 0.14853 m | 0 / 0 |
| 仅启用新回环 | 0.06384 m | 0.04102 m | 0 / 3 |
| 局部配准 + 新回环 | 0.00826 m | 0.01580 m | 20 / 3 |

这些对照使用当前代码开关，**不是重新运行旧版错误回环程序的结果**。
数值仅代表固定种子的合成场景，不能作为整个仿真场地的绝对精度承诺。
结果 `synthetic_results.json`、日志 `check_mapping.log` 和三组单独节点日志。
图示 `synthetic_comparison.png`。

复现：

```bash
source scripts/env.sh
ROS_DOMAIN_ID=99 python3 reports/mapping_consistency_20261102/check_mapping.py
```

## Gazebo 验证

第一条测试路线使用先前示例出生点 (-3.1,1.3)，机器人被场地障碍阻挡，直线阶段超时。
保留 `simulation_route_collision.log` 和 `drive_route_collision.log`，不计为通过。

第二次独立启动，出生点 (-4.8,3.8)，ROS domain 98、Gazebo master 11498。
`drive_simulation.py` 根据轮速里程计控制两组 0.8 m 四边路线，仿真真值只用于记录。
使用 `save_fastlio_pcd:=false`，输出目录指向本报告下的 `sim_map`。

实际保存：46 个关键帧、36 个局部几何因子、5 个已验证回环、11956 个导出点。
日志 `simulation.log`、`drive_simulation.log`；导出文件及关键帧见 `sim_map/`；
顶视图 `simulation_map.png`。图中已覆盖结构的边缘保持集中，但路线未覆盖全场。
真值最终距起点约 0.697 m，说明轮速控制的两圈并非严格物理闭合，
不能把轨迹闭合距离直接当作地图误差，也未对整场绝对精度给出统计结论。
后续的小修改仅增加候选歧义拒绝及日志/default 一致性；完整最终代码另通过合成回归。
测试组 Gazebo 和节点均已正常退出。

运行命令：

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=98
export GAZEBO_MASTER_URI=http://127.0.0.1:11498
ros2 launch robocon_cloud_localization mapping.launch.py gui:=false \
  initial_x:=-4.8 initial_y:=3.8 save_fastlio_pcd:=false \
  map_directory:=/home/alice/rc_2027/cloud_localization/reports/mapping_consistency_20261102/sim_map
# 另一个相同环境的终端：
python3 reports/mapping_consistency_20261102/drive_simulation.py
```

## 其他检查

- Release 编译通过，见 `build.log`。
- CTest：CPU/GPU 全局注册契约和新的建图注册测试均通过，见 `ctest.log`。
  新测试覆盖时间插值/拒绝外推/时钟回退、有效匹配、创新量门限、低重叠拒绝，
  以及“宽半径重叠很高但几何误差过大”的匹配拒绝。
- 现有定位 ROS 回归在单独 domain 101 通过，见 `localization_regression_retry.log`。
  与 Gazebo 并行的首次运行在切换测试节点后未收到定位消息，见 `localization_regression.log`；
  此次单独重试未修改定位算法。未将第一次失败隐藏或计为通过。
- launch 文件和报告测试脚本 Python 语法检查通过。
- 原 PCD 哈希记录在 `map_hashes_before.txt`，最终核对一致。
- 修改前源码保留于 `before/`，差异在 `changes.patch`。`COLCON_IGNORE` 防止备份被 colcon 识别为包。

使用与调参说明：`docs/mapping_consistency.md`。
旧的单张累计 PCD 不会在程序更新后自动去除历史重影，应重新建图并通过 `/save_map` 导出优化结果。
