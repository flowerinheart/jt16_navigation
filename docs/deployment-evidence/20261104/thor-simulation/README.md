# Thor simulation verification — 2026-11-04

Ubuntu 24.04 / ARM64 / NVIDIA Thor，使用`jt16-deps:jazzy-benchmark-aarch64`与独立工作卷`jt16-thor-simulation-20261104`。实际运行Gazebo、传感器桥接、FAST-LIO、AMCL、Nav2及动态障碍预测；结果目录为工作卷内`/workspace/jt16/integration/results/thor-deployment-20261104/`。

## Found and fixed

Thor原始`net.core.rmem_max/wmem_max`为212992字节，本机为4194304字节。JT16单帧原始点云为16×1200×32=614400字节；高负载导航运行中，DDS分片接收不稳定，滚动点云输出停止，TF停更，Nav2无法激活。

| 原始环境 | 结果 | 激活等待 |
|---|---|---|
| Arena small_warehouse / obs05 / seed00 | PROCESS_FAILURE：bt_navigator未激活 | 约60秒后退出 |
| BARN22 / seed00 | PROCESS_FAILURE：未收到AMCL初始定位 | 约60秒后退出 |

临时限制线程可使一个Arena诊断运行成功，但不能修复BARN，未保留该设置。栈采样显示点云适配器等待DDS输入，没有卡在数值计算。只提高UDP缓冲上限后，以下默认线程配置下的导航验证均通过。

`setup.sh`现在调用`configure_ros_network.sh`，将收发缓冲上限至少设为4 MiB，保留已有更大值，写入`/etc/sysctl.d/90-jt16-network.conf`。一次性配置容器增加`NET_ADMIN`，运行容器保持原权限；case与导航参数沿用原配置。

## Actual runs

| 测试 | 整个脚本墙钟时间 | 结果 | 终点误差 | 碰撞 |
|---|---|---|---|---|
| 传感器smoke | 4.14秒 | 40帧点云、100条IMU消息 | — | — |
| BARN22建图 | 61.66秒 | COMPLETE，14/14路径点，地图有效 | ATE RMSE 0.056 m | — |
| BARN22导航 / seed00 | 61.38秒 | SUCCEEDED，scored_success=true | 0.096 m | 无 |
| Arena small_warehouse / obs05 / seed00 | 87.89秒 | SUCCEEDED，scored_success=true | 0.161 m | 无 |

整段脚本时间包括启动、导航/建图、评分与清理；BARN和Arena导航阶段分别为44.19秒、69.85秒。导航仿真超时150秒、进程上限600秒、容器内外层上限900秒；建图使用300秒超时、外层600秒。以上均正常退出，未触发外层超时，退出后测试容器已清理。

原始结果：`sensors.json`、`barn022-mapping.json`、`barn022-navigation.json`、`arena-navigation.json`。修复前失败结果：`barn-before-fix.json`、`arena-before-fix.json`。完整日志、轨迹、生成地图与调试记录保留在Thor的工作卷；主机日志位于`/home/nvidia/jt16-navigation-deploy-20261104T083702Z/.setup/`。

这些是部署后实际运行的代表样例，未重跑全量V1评测；原来的批量评测仍暂停。

原始 JSON 已归档到 `~/.jt16-navigation/evaluations/repository-cleanup-20261110-150638/docs/deployment-evidence/20261104/thor-simulation`，不随 Git clone 迁移。复现入口：`scripts/setup.sh`；容器证据由 `scripts/setup_container.sh` 写入 `JT16_DATA_HOME/reports/evaluation/`。上面的结论与指标可独立阅读。
