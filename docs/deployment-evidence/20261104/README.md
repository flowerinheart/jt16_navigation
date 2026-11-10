# Deployment verification — 2026-11-04

Ubuntu 24.04 / ROS 2 Jazzy。运行配置从独立空工作卷下载固定源码、编译并验证，不复用宿主机安装、评测数据或提取的ROS库。

| 环境 | 配置测试 | Python回归 | C++用例 | ROS接口 | GPU初始化 |
|---|---|---|---|---|---|
| 本机AMD / x86_64 / runtime | 4通过 | 15通过 | 7通过 | 通过 | HIP，1设备 |
| Thor / aarch64 / runtime | 4通过 | 15通过 | 7通过 | 通过 | CUDA，1设备 |
| 本机 / x86_64 / benchmark | 4通过 | 66通过 | 7通过 | 通过 | CPU配置 |

C++由两个gtest程序执行，colcon汇总含包装检查为9项、0失败。benchmark验证在已构建的runtime依赖镜像上执行同一个依赖安装脚本，使用空工作卷下载BARN、Arena和XLeRobot并生成JT16、BARN22、Arena warehouse5样例资源；未运行性能评测。

## 可复现入口

```bash
# 本机：Ubuntu软件源使用USTC镜像。
UBUNTU_MIRROR=https://mirrors.ustc.edu.cn/ubuntu \
SETUP_VOLUME=jt16-clean-runtime-final-20261104 BUILD_JOBS=2 \
bash scripts/setup.sh runtime rocm

# Thor：仓库脚本上传源码后，在目标机原生构建。
bash scripts/deploy_ssh.sh nvidia@100.101.39.17 runtime cuda
```

Thor源码目录：`/home/nvidia/jt16-navigation-deploy-20261104T083702Z`。
Thor工作卷：`jt16-workspace-runtime-aarch64`。
Thor依赖镜像：`jt16-deps:jazzy-runtime-aarch64`，ID `sha256:1e715d666cd60606d4736ca9ac529c645e6479f6c90af0f1146b5bf10ac63935`。

原始检查结果：`rocm-runtime.json`、`thor-cuda-runtime.json`、`benchmark-cpu.json`。完整系统包版本、Ubuntu源与构建日志保存在对应工作卷的`deployment-evidence/`及源码目录`.setup/`。

## 网络与范围

本机Tailscale用户服务已登录，现有Aliyun Shanghai DERP（47.117.137.138:443）已连接，探测延迟约11ms。本机使用已有AMD企业CA验证网络网关证书；SSH经本机SOCKS5端口1055进入Tailnet，Thor当前优先使用点对点连接。

FAST-LIO、AMCL、MPPI和Hesai主链路仍使用CPU；GPU检查验证运行库初始化与设备访问。未验证真实传感器、机器人运动或GPU算法加速；V1历史性能评测保持暂停。

后续Thor实际传感器、建图与导航样例验证及发现的网络配置问题见[仿真验证记录](thor-simulation/README.md)。

原始 JSON 已归档到 `~/.jt16-navigation/evaluations/repository-cleanup-20261110-150638/docs/deployment-evidence/20261104`，不随 Git clone 迁移。复现入口：`scripts/setup.sh`；容器证据由 `scripts/setup_container.sh` 写入 `JT16_DATA_HOME/reports/evaluation/`。上面的结论与指标可独立阅读。
