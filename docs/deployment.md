# Ubuntu 24.04 deployment

默认安装 FAST-LIO、Hesai 驱动、AMCL 与 Nav2。使用仓库脚本从 Ubuntu 24.04 容器构建、验证运行库和已安装节点；支持 amd64 与 arm64 的原生构建。

## 一键配置

宿主机需要 Docker。CUDA 需要宿主机 NVIDIA 驱动与 NVIDIA Container Toolkit；ROCm 需要宿主机 AMD 驱动、`/dev/kfd`、`/dev/dri` 和已安装的 ROCm 运行库。GPU 驱动由宿主机管理。

```bash
# 在已有源码目录执行；自动识别 AMD / NVIDIA / CPU。
bash scripts/setup.sh

# 明确指定运行环境。
bash scripts/setup.sh runtime rocm
bash scripts/setup.sh runtime cuda
bash scripts/setup.sh runtime cpu

# 需要仿真评测才选择 benchmark。
bash scripts/setup.sh benchmark rocm

# 从本机直接传输仓库文件并在 Thor 原生构建。
bash scripts/deploy_ssh.sh nvidia@100.101.39.17 runtime cuda
```

`setup.sh` 构建可复用依赖镜像，将源码复制到独立 Docker 工作卷，调用现有 `fetch_dependencies.sh`、`check_dependencies.py`、`build.sh`，然后运行 Python/C++ 回归与 ROS 节点接口检查。宿主机的 `external/`、build/install、历史结果、仿真资源和评测进程保持隔离。

构建镜像后，`configure_ros_network.sh`将宿主机UDP收发缓冲上限至少设为4 MiB，并写入`/etc/sysctl.d/90-jt16-network.conf`持久化；已有更大值保留。这个一次性配置容器只增加`NET_ADMIN`并挂载对应sysctl目录，运行与评测容器不增加该权限。Ubuntu默认约208 KiB不足以稳定接收约600 KiB点云的DDS分片，实测会造成滚动点云停更及导航激活失败。

两种配置都运行配置测试、动态预测测试、C++插件测试与已安装ROS节点接口检查；benchmark再运行完整66项集成回归，其中地图/评测编排用例需要Pillow和jq。

本地日志在 `.setup/`。工作卷内 `/workspace/jt16/deployment-evidence/` 保存系统包版本、源码清单哈希、架构、ROS 包路径和 GPU 枚举结果。`SETUP_VOLUME` 可指定全新工作卷；`BUILD_JOBS` 默认2，限制编译内存占用。

构建验证默认使用ROS domain 197及`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，避免与宿主机任务互相发现；实际跨机器ROS通信需配置对应domain和发现范围。

可用 `UBUNTU_MIRROR` 指定 Ubuntu APT 镜像地址，例如 amd64 使用 `https://mirrors.ustc.edu.cn/ubuntu`；arm64 对应 `ubuntu-ports`。镜像仍按Ubuntu签名校验；失败安装最多重试三次，重试复用已下载的软件包。构建支持Docker传统构建器，无需额外安装buildx。

```bash
# 明确从空工作卷验证，不使用本机现有安装或构建缓存。
SETUP_VOLUME=jt16-clean-test BUILD_JOBS=2 bash scripts/setup.sh runtime rocm

# 进入脚本输出的镜像与工作卷。
docker run --rm -it --network host \
  -v jt16-workspace-runtime-aarch64:/workspace \
  jt16-deps:jazzy-runtime-aarch64 bash
source /opt/ros/jazzy/setup.bash
source /workspace/jt16/install/setup.bash
```

进入需要 GPU 的交互容器时，沿用 `setup.sh` 中该 backend 的设备与运行库参数。

## 依赖配置：七类

| 类别 | 默认运行配置 | 配置位置 |
|---|---|---|
| OS／编译工具 | Ubuntu 24.04、GCC、CMake、colcon、Git、vcstool | `config/setup/image.env`、`runtime.apt` |
| ROS 组件 | ROS 2 Jazzy、Nav2/AMCL/MPPI/Smac、Cyclone DDS、消息与launch | `config/setup/runtime.apt` |
| 外部源码 | 3个仓库：FAST_LIO_Hesai、HesaiLidar_ROS_2.0、Nav2；SDK由驱动固定子模块引入，Nav2仅构建bringup配置包 | `dependencies.deploy.repos` |
| C++／Python 库 | Eigen、PCL、Boost、yaml-cpp、SSL、pcap、NumPy、SciPy等 | `config/setup/runtime.apt` |
| GPU 驱动／运行库 | CUDA：NVIDIA容器运行时注入；ROCm：挂载宿主机ROCm库与AMD设备 | `scripts/setup.sh`、`check_deployment.py` |
| 评测数据集 | 默认不下载；benchmark才下载BARN和Arena两个固定版本仓库 | `dependencies.benchmark.repos` |
| 模型／资源 | 当前主链路没有需要下载的神经网络权重；benchmark下载固定版本XLeRobot URDF/mesh，再生成JT16仿真模型 | `dependencies.benchmark.repos`与模型生成脚本 |

`benchmark.apt` 增加 Gazebo Harmonic、ros_gz、绘图、视频和评分工具；默认运行镜像不安装它们。旧 `dependencies.runtime.repos` 中 small_gicp、KISS-Matcher，以及 reference/templates 的设计参考仍可按旧脚本下载，但不属于当前运行链路。

官方 `nav2_bringup` APT包强制依赖TurtleBot仿真和Gazebo，因此默认配置从固定Nav2源码构建该配置包（关闭其lint测试），算法库继续使用APT；导航与定位launch可运行，TurtleBot仿真launch留给可选评测环境。

基础 Ubuntu 镜像使用 digest，外部 Git 使用完整 commit，子模块跟随父仓库固定版本。APT 包按 Ubuntu/ROS 仓库解析，实际版本记录进依赖镜像；保存构建后的镜像可复用同一组系统依赖。该构建不使用本机 `integration/deps` 提取库兜底。

## ROCm 与 CUDA 的范围

当前 FAST-LIO、AMCL、MPPI 和 Hesai 数据解析使用 CPU。两种 GPU 环境共享同一份源代码和依赖镜像；配置检查会实际初始化 HIP 或 CUDA 驱动并枚举设备。Hesai 可选 CUDA 分支固定关闭，它默认指向旧架构，不能用来证明 Thor GPU 加速。

Thor 上直接构建 arm64，不运行 amd64 镜像模拟。NVIDIA 驱动、JetPack 与 Container Toolkit 必须匹配宿主机；无需为这条 CPU 主链路安装 CUDA 编译器。ROCm 默认路径 `/opt/rocm`，可通过 `ROCM_PATH` 指向宿主机安装。

## 使用真实机器人

构建与接口检查不会发送机器人运动命令。启动实际系统前，需提供传感器网络、JT16真实扫描参数、外参、轮式里程计/IMU输入和导航地图。使用官方驱动/FAST-LIO launch及Nav2配置；仿真专用IMU和instant-cloud适配器不应替代真实硬件输入协议。

历史 V1 性能评测仍暂停；部署验证与性能验收分别记录。

2026-11-04本机ROCm、Thor ARM64/CUDA及可选benchmark配置验证结果见[部署验证记录](deployment-evidence/20261104/README.md)。

Thor实际仿真样例、UDP缓冲修复前后结果见[仿真验证记录](deployment-evidence/20261104/thor-simulation/README.md)。
