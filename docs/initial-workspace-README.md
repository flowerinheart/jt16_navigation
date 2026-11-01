# LiDAR SLAM workspace

源码下载目录；尚未安装系统依赖、构建或运行验收。

| 目录 | 用途 |
|---|---|
| FAST_LIO_Hesai | 禾赛 JT16 FAST-LIO2 前端，ROS2 分支 |
| HesaiLidar_ROS_2.0 | 禾赛驱动，包含 SDK 子模块 |
| XLeRobot | 官方机器人模型与控制参考 |
| Lidar_nav2_ws | 仿真、地图、重定位、Nav2 集成参考 |
| evo | 轨迹评分工具 |
| KISS-Matcher | 全局点云配准参考 |
| small_gicp | 局部点云精配准 |

具体版本见 `sources.lock.json`。Lidar_nav2_ws 上游有 gitlink 但缺少 .gitmodules；已按上游记录的提交补齐 src/localization/rtabmap 和 src/registration/KISS-Matcher。后者使用本目录 KISS-Matcher 克隆作为下载来源。

Nav2、ros_gz 优先采用 ROS 发行版软件包，本次未安装。未下载 benchmark 大数据集或预训练模型。Hesai FAST-LIO2 与导航参考主要针对 Humble；Jazzy/Harmonic、JT16 仿真输入及 XLeRobot 接口仍需适配。导航参考的仿真时钟配置也需统一，不能未经验证直接用于验收。

New owned integration Git repository: jt16_navigation/. Architecture, pinned dependencies and build instructions are documented there. Existing integration/ remains the historical sensor prototype. Jazzy template sources are downloaded under references/.
