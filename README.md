# Robocon 2027

ROS 2 仿真、点云建图、重定位与导航代码，来自 `~/rc_2027` 的当前源码快照。

| 目录 | 内容 |
|---|---|
| `ABU-Robocon-2027-Gazebo/` | Robocon 2027 场地模型与 Gazebo 资源 |
| `Robocon2027/` | 机器人仿真、建图相关 ROS 2 工作空间与模型 |
| `cloud_localization/` | MID360 仿真、FAST-LIO、局部子图配准、回环优化、GICP / 3D-BBS 重定位 |
| `rc_navigation/` | 导航相关工作空间、配置和地图资源 |

各目录的启动方法和依赖以对应 README 为准。构建 ROS 2 工作空间后，在该工作空间根目录加载环境：

```bash
source install/setup.bash
```

该命令只加载环境。建图、定位和导航需再执行对应的 ROS 2 启动命令。

本仓库直接保存各嵌套项目的源码和本地修改，不依赖 Git 子模块初始化。
第三方代码的来源版本记录在 [THIRD_PARTY_SOURCES.md](THIRD_PARTY_SOURCES.md)，
各项目原有许可证和版权声明保留在对应目录。
`build/`、`install/`、运行日志、缓存和重复的工作空间压缩包不纳入版本管理；
换机使用时需要重新安装依赖并构建。
