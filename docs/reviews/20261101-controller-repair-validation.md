# 小集修复与控制器对照

## 当前结论

默认保留 DWB。MPPI 已获准提前对照，早期候选在仓库任务中发生实际
动态碰撞，未采用。后续发现两种控制器此前都没有得到有效局部障碍
代价，故早期对照不用于判断修复后的控制器优劣；正在重新对照。
尚未宣称冻结小集或 V0 全部验收通过。
没有运行全量评测，没有提交代码，也没有使用 GT 导航旁路。

## 本轮修复

- 修正运动学仿真底盘的摩擦：固定关节模型没有轮端驱动力，贴地碰撞盒
  的默认摩擦使纯转向命令无法形成实际角速度。保留重力和碰撞盒，仅将
  底盘碰撞的两个摩擦系数设为零；使用默认碰撞名称，确保 sdformat
  实际保留这些系数。这不是实机轮动力学验证。
- `/odom` 改为 reliable 发布，匹配 Nav2 的可靠订阅；输入 FAST-LIO
  `/Odometry` 继续使用传感器 QoS。安装接口测试也采用 reliable `/odom`
  订阅，检查位姿、派生速度和 TF。
- 保存带仿真接收时间的三段速度命令、带消息时间的 `/odom` 速度及扫描。
  去掉逐帧高带宽 DWB 评分订阅，动态障碍消息队列由 10 增至 100。
  观测缺失仍判不可评分，不放宽 0.25 秒最近观测时间门槛。
- 建图及导航单例在启动前拒绝非空输出目录；批量调用不再提前在目录内
  创建 `runner.log`。历史失败记录保留。
- MPPI 参数从已安装 Nav2 示例生成，只替换 FollowPath 控制器；沿用
  DWB 的速度、加速度限制和矩形 footprint，模型步长匹配控制频率。
  实际执行参数作为输入归档，默认控制器仍为 DWB。

## 诊断证据

- `arena-command-probe.NbKALn/.../navigation-sensors.npz`：终点三段命令
  均约 -0.2105 rad/s，而实际里程计角速度接近零；终点扫描最近距离
  0.869 m，无 0.35 m 内自车假点。定位到仿真命令响应而非 DWB 不转向。
- `arena-floor-friction-repair.9CaqPF/.../nav2-goal.json`：Nav2 已在
  63.24 秒完成，但动态观测有空洞，scoring_valid=false；不计成功。
- `controller-comparison.6PwXWY`：保留 QoS 修复前的对照，不用于晋升。
  日志明确报告 Nav2 无法订阅 best-effort `/odom`；MPPI 平均线速度仅
  0.0455 m/s，缺速度反馈时反复从零速度计算受加速度限制的首步命令。
- CycloneDDS 的 TF 消息发布标识与节点图标识不匹配，审计保持失败。
  FastDDS 独立发布者探针能正确识别两个发布者，因此当前验证采用
  `rmw_fastrtps_cpp` 和 LOCALHOST discovery，不修改所有权判定。

## 同条件仓库单例对照

根目录：`integration/results/v0/controller-qos-repair.s2si8i`。
场景 small_warehouse / 5 障碍 / seed 1，超时 150 秒；相同传感器、
FAST-LIO、地图、任务起始 actor epoch 协议和评分。

| 控制器 | Nav2 状态 | 严格成功 | 耗时 s | 最小动态间隙 m | 终点距离 m | 碰撞 |
|---|---|---|---:|---:|---:|---|
| DWB | SUCCEEDED | true | 62.220 | 0.103222 | 0.177721 | false |
| MPPI | SUCCEEDED | false | 76.364 | -0.185393 | 0.043683 | true |

两个结果均 scoring_valid=true、localization_success=true、TF 审计通过。
单例不代表完整小集成功率；MPPI 终点距离更好不能抵消碰撞和更长耗时。
本轮不采用此 MPPI 参数，也不声称所有 MPPI 配置都弱于 DWB。

## 已执行验证

- `PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -q`：34 项通过。
- `bash scripts/build.sh scaffold`：通过。
- sourced ROS 与 install 后，LOCALHOST discovery 下的
  `integration/tests/ros_interface_smoke.py`：reliable `/odom`、速度、TF、扫描通过。
- owned integration shell `bash -n` 与 Python AST 检查通过。
- 针对控制器参数生成、桥接、评分器、actor 适配器、接口 smoke 和
  TF C++ 探针的 lizard 检查：无 CCN/长度/参数数量阈值超限。
- 仓库 DWB 的 33 项评分指标已从归档源码、实际机器人和障碍观测、
  归档碰撞世界独立离线重放，差值不超过 1e-12。
- 同条件 DWB 与 MPPI 的 source/runtime/input provenance 校验通过。

## 剩余验证

进一步逐个障碍比较发现，五组差异很大的控制参数都在 mission+32 秒
撞上 actor0。因此重新检查实际 costmap，不再假定控制器已经看到障碍。
`costmap-height-probe.IEEtou` 是明确的 45 秒诊断，不是冻结小集评测。
现场查询显示：base_link 在 odom 中 z=-0.366..-0.371 m，扫描有
66–140 个三米内点、最近距离低至 0.2363 m，但 GetCostmap 返回
max=0、lethal=0。Nav2 将扫描投为 z=0 平面后，坐标变换到初始 IMU
原点的 odom，点高度均为负；默认 min_obstacle_height=0 把点全部删除。
源码确认观测缓冲和障碍层标记更新都有此过滤。

已同时将 local/global 的 obstacle_layer 及 scan 观测源下限设为 -0.5 m。
这是二维表达坐标修复：上游仍按真实 base-relative z=0.02..0.35 m
筛选原始三维点，不接纳地面点，也不更改评分。新增里程计完整三维
位姿归档，防止今后只看 xy/yaw 漏掉此问题。配置回归测试约束两处
下限一致，并覆盖当前 IMU 安装高度；34 项 UT 和构建通过。

`costmap-height-repair.P3eXBL` 是相同 map2 的正式 150 秒候选。
现场 GetCostmap 查询恢复 max=254，86–124 个 lethal 单元，证明障碍
真正进入 costmap。最终 106.522 秒到达，但原评分报预热观测缺口；
后续诊断发现所有 17 个未匹配机器人样本均早于 mission_start=10.642，
而任务期间障碍最大观测间隔仅 0.304 秒。评分器原先错误地将导航节点
启动预热也纳入碰撞/运动评分。
已按实际发布的 mission_start 划定导航评分窗口，完整原始轨迹仍保存，
全时段 GT 仍用于初始化定位质量检查。新增回归覆盖预热缺观测、任务
期间缺观测、真实碰撞和空任务窗口，35 项 UT 通过。诊断重算该历史
记录仍 collision=true（最小 -0.200819 m）、scored_success=false；
不改写历史 JSON，不放宽任务期间 0.25 秒最近观测门槛。
`mppi-visible-costmap-pilot.LUEKKC` 在同一 map2 中试验已授权的 MPPI：
105.026 秒 SUCCEEDED，scored_success=true，零静态/动态碰撞，定位、
TF 和 provenance 校验通过。终点距离 0.068380 m，最小动态间隙
0.020813 m。对每个机器人样本同时检查 0.25 秒内前后两个实际障碍
观测，最小间隙仍为正（0.020744 m），没有依赖未来/过去配对差异刷绿。
保留修正后的真实局部障碍输入，暂未改变默认 DWB，正在继续相同
参数的仓库单例；其结果 93.330 秒到达但发生 actor1/actor3 动态碰撞，
最小间隙 -0.264987 m，故未继续 map5，也未采用该候选。
检查 Nav2 原生 CostCritic 后，将 near_collision_cost 从 253 调至 120：
在当前 inflation/footprint 下，约在车体外 0.25 m 处开始严重软代价，
而不等车心进入 inscribed 区域。保持速度、加速度、路径和所有评分
不变，`mppi-early-clearance-pilot.0nTL0y` 仓库结果 106.420 秒到达，
scored_success=true、零静态/动态碰撞，最小间隙 0.110801 m，终点
距离 0.067395 m，定位、TF、provenance 通过。随后只用同一配置重跑
map2；其结果 107.338 秒到达但动态碰撞（最小 -0.140902 m），因此
未继续 map5。
又确认控制器预测转向上限 0.8 与速度平滑器实际 0.6 不一致；统一为
0.6，并将 SlowCircle 改为原生 limit（linear=0.1、angular=0.6）。
limit 仍保持曲率联动缩放，不是独立只限制线速度；但低于限速或纯
旋转不会像旧 slowdown 一样继续统一乘 0.3。车体碰撞预测保持启用。
`mppi-command-consistency-pilot.BxITva` 在同一 map2 中 99.960 秒到达，
但动态碰撞，最小 -0.243623 m，未通过。随后仅将原生 CostCritic 的
cost_weight 从 3.81 增至 12.0，让已经出现的危险代价有足够影响力；
`mppi-clearance-weight-pilot.1s4GPh` 的 map2 结果 103.326 秒到达，
scored_success=true、零碰撞，最小动态间隙 0.132307 m，终点距离
0.115031 m，定位、TF、provenance 通过。该配置仓库结果 106.182 秒
到达、零碰撞、最小间隙 0.125991 m，但 localization_success=false：
末端 AMCL stamp=118.909917，而收集到的最后 GT stamp=118.898，
因此被原判定视为未来数据（-0.011917 秒），并非位姿误差超标。
已有 refreshLocalization 改为等待新 AMCL 且真实 GT 覆盖其时间，
不修改任何观测时间、不放宽 1 秒 freshness 或 0.1 秒匹配门槛。
回归测试让 AMCL 先到、GT 下一轮才到，验证继续等待覆盖；35 项通过。
`mppi-terminal-sync-pilot.SWjNhd` 在同一仓库单例重跑，保留旧失败记录。
该次最终仅 3.060 秒即 PROCESS_FAILURE：FAST-LIO odometry stale；
GT 持续正常，LIO、桥接 odom、body scan 同时滞后，不能算终端同步已验证。
`upstream-latency-probe.gZUE8B` 是启动命令误用 zsh 导致环境加载失败的记录，
已停止，只保留，不计入候选结果。
随后 `upstream-latency-probe.hiOYyf` 为 NAV45 短诊断，预期 TIMEOUT、
零碰撞、input_failure=null，不属于 NAV150 验证集成功。
旁路记录显示 raw cloud 614400 bytes/frame、约 98 Hz；IMU 200 Hz，
raw 最大 gap 0.030 秒，assembled/LIO 最大 gap 0.170 秒，
LIO 最大接收 age 0.018083 秒，未复现早先 stale 故障，不能宣称修好。
完整统计保存在该候选根目录 transport-probe.json。
已补严格 elapsed 为有限非负且不大于任务 timeout 的成功门槛，
并将实际 MPPI/DWB/costmap/planner/AMCL/collision-monitor 库加入 runtime hashes；
35 项回归通过，覆盖 deadline 边界、非有限值和库身份篡改。
`mppi-upstream-instrumented-pilot.nFpBqe` 完成同一 NAV150 仓库单例采集，
保持 near120/weight12/w0.6/limit 和终端 GT 同步，不放宽 .5 秒输入门槛。
结果 TIMEOUT 150.042 秒，零碰撞、定位通过、input_failure=null，
目标距离 4.156560 m，最小间隙 0.001376 m，严格失败；TF/provenance 通过。
130 秒旁路记录：IMU 最大 gap .006 秒；raw 最大 gap .04 秒；
assembled/LIO 最大 gap .20 秒；LIO 最大 age .074083 秒，
未复现 stale，但不能由此宣称间歇故障已修复。
40–110 秒 cmd_vel_nav 约 34.4% 倒退、27.8% 近零线速度，
该窗口 cmd_vel 与 cmd_vel_nav 基本一致，安全监控不是主要减速来源。
下一候选仅将原生 PathFollowCritic.cost_weight 从 5 提至 10，
保持 CostCritic 与所有硬碰撞/评分门槛，仍只试仓库，不推进完整验证集。
`mppi-progress-pilot.DQPfmV` 104.822 秒到达、定位通过，但原评分为
observation timestamp gap：仅 actor1 的末端一个任务后 GT 样本距其
最后观测 .278 秒，任务结束前所有样本均在 .25 秒以内。
已明确记录导航结束 ROS clock，collision/motion/goal 使用起止任务窗口，
完整 GT 仍保留并用于终端定位刷新，elapsed 改为该真实 clock 的任务时长。
新增回归覆盖任务后缺观测不污染任务评分、任务内真实碰撞仍被拒绝。
诊断按旧 elapsed 导出的 end=117.742 离线重放仍有真实动态碰撞，
minimum=-.272879 m，scored_success=false；历史 JSON 未改写。
因此撤回 PathFollow 10，恢复官方 5。下一候选只将 CostCritic 普通权重
12 降至 6，同时 critical_cost 300 提至 1000：普通场域惩罚减半，
near120 区域惩罚反而更强，硬碰撞、速度和评分不变，尚未验证。
该候选 `mppi-task-window-pilot.42cutD` 最终 TIMEOUT 150.068 sim 秒，
wall 250.486 秒，RTF=.594；零碰撞、minimum=.017379 m，目标距 .350993 m，
定位因最后 AMCL 127.499917 对 GT 159.358 陈旧而失败，不是误差超限。
TF/provenance 通过。末端 130–160 秒 cmd_vel_nav 接近零，扫描最近约 .72 m；
GT=(2.027,-9.384,-1.298)，goal=(1.73,-9.57,-1.289)，
已经对齐最终朝向却保留横向位置误差。
原生 GoalAngleCritic 在 .5 m 内启动，而位置 GoalCritic 权重5、朝向权重3，
非完整约束车的横向修正必须先偏离最终朝向，容易形成局部极小。
下一候选只将 GoalAngleCritic.threshold_to_consider 对齐既有 xy .1 m，
让原生 stateful goal checker 先完成位置，再对齐朝向；成功容差不变。
`mppi-goal-angle-pilot.HiwpeU` 150.086 秒 TIMEOUT，零碰撞、
minimum=.134720 m、定位通过、目标距 .230989 m，仍未通过。
末端 cmd_vel_nav 近零，扫描最近约 .85 m；AMCL=(1.888,-9.497,-1.224)，
到目标位置仍需明显转向。PathAngleCritic 同样在 .5 m 内退出，
使位置尚未到位时缺少朝向路径点的引导。
下一候选保持以上修复，将 PathAngleCritic 启用至 xy .1 m，
max_angle_to_furthest 从 1.0 降至 .4 rad；不改变任何成功容差。
`mppi-terminal-bearing-pilot.JP3lqN` 仓库严格通过：122.192 秒 SUCCEEDED，
scored_success=true、零碰撞、minimum=.156675 m、goal=.059657 m，
localization=true、input_failure=null。TF/provenance 验证和 33 项归档
指标离线重放通过。该修复才有完整任务窗口+终端同步的成功证据。
继续同一源码、同一 mppi-params.yaml 的 map2/obs5/seed0；
仍未声称 Arena3 整体通过、未切换默认入口、未跑全量。
随后同版本 map2 严格通过：104.494 秒，goal=.085527 m，minimum=.125138 m；
map5 严格通过：82.400 秒，goal=.049950 m，minimum=1.540977 m。
三例均零碰撞、定位通过、无输入故障，33 项各自离线重放及 TF 通过，
三例 sources/runtime_hashes 完全相同，候选 Arena Quick3=3/3。
按用户授权将该 MPPI 接入 Arena/BARN/nav-smoke 默认入口：在任何子进程
启动之前，从安装 DWB 机器人模板生成 MPPI 参数，显式 NAV2_PARAMS_FILE
仍可覆盖，实际参数随输出归档。未复制完整 MPPI YAML，不新增依赖。
nav-smoke 也拒绝非空输出。36 项 UT、bash -n、scaffold build 通过。
最终复跑根目录 `mppi-default-small-suite.CjYMCH`，从默认入口重新运行，
不把采用前成功混入最终结果；之后还需该冻结源码的 BARN3 建图/导航。
最终仓库复跑 133.454 秒 SUCCEEDED，但 scored_success=false，
真实 actor0 动态碰撞，minimum=-.166544 m；goal=.104237 m，
定位通过、input_failure=null，TF/provenance 通过。未继续 map2/map5/BARN。
首次重叠 stamp=62.288，最严重=64.056：碰撞前2秒 actor 在车体侧方
(-.034,-.749)m，scan 最近=.451m，cmd vx=.35；前.5秒 scan=.340m，
cmd 已限至 .10，但 actor 继续侧向靠近，首次重叠时 scan=.266m；
之后机器人几乎停车，actor 仍进入车体包络。
scan 最大 gap=.18 秒，LIO 最大 gap=.41 秒，未触发输入陈旧；
此时不是完全漏检，而是即时成本/静态相对运动预测不能表达侧向逼近。
采用前的3/3并未被改写，但不能用来掩盖默认入口复跑碰撞。
请求用户选择是否增加不依赖识别模型/GT 的轻量 LiDAR 障碍运动估计与
短时预测 costmap，或仍限制为即时 costmap 原生策略；尚未新增此模块。
MPPI 默认入口只是开发候选，不是交付通过。最终独立 delivery-review
和 ponytail-review 尚未签发，Goal 未完成，未执行全量或实机评估。
继续限定原生策略审计：实际 actor0 velocity 约(.04,-.19)m/s；
机器人在追越时 x 从 -3.252 变为 -3.391，朝 actor 所在侧转弯。
旧 .45 m 限速圈直到 scan=.34m 才稳定将 vx 限至 .1，介入过晚。
在未获新增预测模块确认前，原生候选只将 SlowCircle.radius 扩至 .65m，
linear=.1、angular=.6、FootprintApproach 和评分均不变。
新增固定侧方 .55m 四回波配置回归，旧 .45m 无法覆盖，36 项 UT 和
scaffold build 通过。`mppi-early-side-limit-pilot.8KKZc2` 仅跑仓库
NAV150/obs5/seed1，尚未声称成功、未继续全量。
该候选最终严格通过：119.168 秒，zero collision，minimum=.125597 m，
goal=.136634 m，localization=true、input_failure=null，33 项离线重放、
TF/provenance 均通过。当前仍只在原生 MPPI/Collision Monitor 内修复，
未新增预测/识别模块。`side-limit-small-suite.Q6178m` 冻结同一源码，
先重复仓库任务，验证不再复现采用前3/3后碰撞的问题，再推进小集。
同版本仓库复跑 117.130 秒严格成功，minimum=.142568 m，goal=.128600 m，
零碰撞、定位通过、无输入故障，provenance 通过；当前仓库连续2/2。
继续 `side-limit-small-suite.Q6178m/arena/map2_obs05/seed_00`，默认 MPPI，
同一 .65m 原生限速圈；不复用 .45m 历史候选结果。
该 map2 109.156 秒到达但 collision=true，minimum=-.226801m，
定位通过、input_failure=null，严格失败，不继续其 map5/BARN。
用户已明确授权轻量 LiDAR 运动估计+预测 costmap，不增加识别模型或GT导航。
新增 `motion_prediction.py`/`predict_obstacles.py`：只读真实 `/scan` 和
FAST-LIO `/odom`，按原扫描时间严格匹配位姿并补偿 ego motion，
跟踪紧凑几何簇，通过短历史拟合速度，输出1.6秒恒速扫掠占据和速度记录。
新增原生 Nav2 `PredictionLayer`，仅在 local/odom costmap、inflation 前添加
预测占据，整个局部窗口重组以清除过期预测，不覆盖真实障碍层；
超过 .5秒或非法/错误坐标系输入使该层 current=false。
实际源数据/预测时间戳、点、速度均归档，预测断流同样判输入故障。
初版离线仓库3363条移动记录中1838条远离所有真实 movers（仅诊断用GT）；
直线表面法向证据约束+历史点云“运动解释优于静止解释”后，
1350条中仅48条远离 movers，同时仍有碰撞前57条 actor0 估计。
在线算法从不读取此GT诊断，也不硬编码actor尺寸、身份、路径或速度。
44项数据回归通过，原生 C++ costmap测试和安装接口测试通过；
覆盖静态墙/墙角可见变化、运动簇、ego补偿、失踪、非法时间/点云、
预测清除、旧预测过期和保留真实障碍成本。新增函数CCN最大13，nloc<80。
colcon测试必须限定 `--base-paths src`，避免历史 executed-inputs 包重复发现。
尚未运行新预测版本的导航成功证据，不得据离线估计/UT声称交付通过。
`lidar-motion-prediction-pilot.Hs5FgR` map2 首次严格通过：131.334 秒，
minimum=.263793 m，goal=.081115 m，零碰撞、定位通过、input_failure=null。
motion_prediction_enabled=true，1335帧预测心跳（max gap=.130000秒），
150096个预测点、1271条速度记录；33项离线评分重放、TF/provenance通过。
新的 `lidar-prediction-small-suite.Dk5ry2` 用同一源码/默认参数先复跑
map2，再推进剩余固定小集，不能用一次成功代替最终交付。
预测版同源码 map2 复跑再次严格通过：137.842秒，minimum=.244722m，
goal=.028237m，零碰撞、定位通过、input_failure=null；1390帧预测心跳，
157152预测点，provenance通过，当前map2连续2/2。继续同源码仓库，
仍未完成整个Arena3/BARN3和最终交付评审。
同冻结源码仓库预测版严格成功：108.098秒，minimum=.157355m，
goal=.094482m，零碰撞、定位通过、input_failure=null；1098帧预测心跳，
181792预测点，provenance通过。继续同源码 map5/obs5/seed1。
预测版 map5严格成功：85.420秒，minimum=1.533034m，goal=.073080m，
零碰撞、定位通过、input_failure=null；862帧预测心跳、18784预测点。
该预测候选正式Arena3=3/3，同源码map2另有一次严格复跑成功。
为最终交付预检，提取评分器任务窗口过滤为独立同语义 helper（旧函数CCN15），
另修复audit_v0单行def令lizard误计模块代码的问题（AST/radon确认实际CCN1）。
45项UT、所有owned integration脚本/CPP函数CCN<15/nloc<80预检通过；
候选3例各33项离线指标在重构后逐项完全一致，不改写旧JSON。
最终源码身份已变化，候选3/3不混入最终新源码交付结果。
先用 `prediction-dwb-comparison.hmgDtr` 同输入链/同预测层的DWB显式参数
对照map2，按结果决定默认控制器后再冻结最终Arena+BARN小集。
该DWB 105.658秒到达、定位通过、无输入故障，但scoring_valid=false：
仅actor0在任务中段GT=42.670处有一个 observation timestamp gap，
并非终端缺观测，不能以此判定DWB控制器更差，历史JSON保留原失败。
保留原 `.2秒` Set/Get运动更新及pending语义，另增独立`.1秒`只读Get采集，
两种future的pending独立，均只发布RESULT_OK的真实状态，不用计划坐标或插值。
新增回归验证只读请求不会释放/改变运动pending，拒绝无效状态；46项UT、
函数复杂度预检通过。评分 `.25秒` 门槛、场景/路径/速度/epoch未放宽或改写。
`prediction-dwb-ground-truth-repair.sHiOJJ` 同一DWB/预测参数重跑map2，
先验证采集完整性再作控制器对照，不将未评分的旧结果当作成功。
该复跑109.334秒SUCCEEDED，8363条真实动态观测，scoring_valid=true、
localization_success=true、input_failure=null，但dynamic_collision=true，
minimum=-.054732m，严格失败；源码/runtime/input provenance校验通过。
保留此失败，不将动作到达当作成功，也不使用旧缺观测结果比较控制器。
默认继续MPPI，使用新根 `lidar-prediction-final-small-suite.ur2Ja8`，
先复跑map2，同冻结源码通过后才推进剩余Arena/BARN；当前终态未达成。
该默认MPPI复跑150.024秒TIMEOUT，目标距离.034607m，但动态碰撞
minimum=-.195492m；定位/评分有效、input_failure=null，严格失败。
最差碰撞actor1、GT=51.374，机器人自48.87起已停住；48.37..50.37
仍有该障碍附近的LiDAR速度估计，说明停止不能防止侧向障碍撞入。
不继续剩余小集。下一候选将恒速预测1.6秒延到4秒以覆盖MPPI约3.7秒
rollout，并复用已安装Nav2 RotationShim在不变xy/yaw门槛内完成终点转向。
没有新增识别/GT控制，保留速度/场景/种子/150秒门槛。
新增配置断言和预测窗口回归，46项UT、scaffold构建、复杂度预检通过。
`lidar-prediction-horizon-shim-pilot.EQuQD9` 只复跑map2；尚无成功结论。
首次启动PROCESS_FAILURE：AMCL configure的change_state响应DDS发送超时，
localization manager未进入activate，全局costmap缺map变换；FAST-LIO正常。
安装接口测试通过。保留该进程失败，不排除出交付风险；同源码用
`retry/map2_obs05/seed_00`新目录重新启动，仅诊断可复现性。
同源码重试正常激活，但83.178秒FAILED，动态minimum=-.168105m，
定位/评分有效、无输入故障，轨迹多次Optimizer fail：延长硬占据未修复。
静态扫掠硬障碍把未来撞入当前位置也当作立即硬碰撞，可能拒绝离开轨迹。
下一候选把预测改为最高252的软占据，复用现有InflationLayer.computeCost
和半径产生风险扩散，max合并保留真实障碍254；真实LiDAR硬检查不变。
原生集成测试新增软扩散、重叠真实lethal保留断言，46项UT、构建、
原生测试及复杂度检查通过；4秒预测/RotationShim保持，未放宽评分。
`lidar-soft-prediction-pilot.BvFI4T`只跑map2，当前无通过结论。
软预测pilot严格通过：110.844秒，goal=.099145m，minimum=.307365m，
零碰撞、scoring/localization有效、input_failure=null，1097预测心跳。
源码/runtime/input校验通过。单次成功不等于小集交付，接着同源码复跑。
`lidar-soft-prediction-final-small-suite.aYJeYc`默认map2复跑严格成功：
129.718秒，minimum=.356214m，goal=.054309m，零碰撞，定位/评分通过，
无输入故障；同源码热点连续2/2。继续同源码仓库，未跑全量。
同源码仓库严格成功：120.224秒，minimum=.354697m，goal=.095376m，
零碰撞、scoring/localization有效、input_failure=null。继续map5固定样例。
同源码map5严格成功：79.812秒，minimum=1.393750m，goal=.094278m，
零碰撞、定位/评分有效、无输入故障。最终根Arena Quick3=3/3，另有
同源码map2 pilot严格通过。接着冻结BARN22/55/77，不扩展全量。
BARN采集由GT驱动规定探索路径，不声称自主探索；Nav2导航仍不用GT。
该源码BARN22建图完成14/14，ATE=.057381m、地图F1=.885821；导航
150.008秒TIMEOUT，goal=6.110148m、零碰撞，但终端AMCL不新鲜而失败。
停滞约55秒开始，最后30秒仅移动.090m；附近scan min约.65m，没有
运动预测（全任务仅39条tracks，停滞段0）。定位位置误差仅.031m。
暂停55/77，最终整体闸门未通过。读取原生CostCritic确认>=120的普通
膨胀区代价被统一critical1000，疑似窄道前进代价平台导致停滞。
下一候选仅near_collision_cost=252，对齐预测核心软代价，保留真实
障碍硬检查/速度/安全监控/150秒及评分；46项UT通过。
`lidar-soft-prediction-barn-cost-pilot/world_022/navigation/seed_00`使用
原已生成地图作诊断，不当作同源码最终建图证据；尚无终端结果。
near_collision_cost=252的BARN22对照严格通过：65.744秒，minimum=.166859m，
goal=.191465m，零碰撞、定位/评分有效、input_failure=null；支持代价平台
停滞诊断。重新冻结源码，新根跑Arena3+BARN3/建图，每例失败即停止。
`lidar-graded-prediction-final-small-suite.uUfX2S`在首例map2即严格失败：
150.072秒TIMEOUT、goal=9.744308m、dynamic minimum=-.161311m；定位/评分
有效，无输入断流。验证流程自动停止，未执行后续例。252参数不得采用。
已恢复near_collision_cost=120及对应断言，恢复原Arena3通过版本身份，
保留其BARN22失败，不拼接252的BARN成功。当前没有同一配置两个小集通过。
下一方向需要时间相关的动态轨迹代价，而非静态未来扫掠区域；这将
新增MPPI critic扩展，超出本轮轻量预测costmap实现，先请用户选择授权。
最终独立delivery/ponytail评审未签发，Goal仍未完成；没有全量、commit。
自动续行先验证现有原生参数替代方案，不添加尚未获批的MPPI critic。
`inflation-gradient-pilot.oq5WPj/candidate-params.yaml`仅将local/global
cost_scaling_factor=2改4，完整配置逐项相等断言通过，其余参数未变。
near_collision_cost=120、真实硬检查、膨胀半径.75、安全监控、速度、
150秒门槛保留。先用已有地图对照BARN22，默认源码/安装配置未改；
通过后才验证map2，不混入旧版最终证据。当前运行未结束。
衰减4候选BARN22严格通过：126.126秒，minimum=.252021m，goal=.132319m，
零碰撞、定位有效、input_failure=null。接着同一参数map2验证动态安全；
不能以BARN单例通过取代Arena门槛，默认仍未修改。
衰减4候选map2严格通过：111.102秒，minimum=.012859m，goal=.110346m，
零碰撞、定位有效、input_failure=null。但安全余量仅1.3cm，不能认定
可靠修复；先用同参数 `repeat/arena/map2_obs05/seed_00` 复跑，未推进全套。
衰减4候选map2复跑严格成功：128.674秒，minimum=.058401m、goal=.043330m，
零碰撞、定位有效、input_failure=null。热点同参数2/2，另有BARN22成功。
将local/global衰减4设为默认候选（其余参数不变），更新不准确的旧注释；
重新构建、冻结源码，接着新根完整Arena3+BARN3，不复用旧源码证明。
动态安全余量1.3/5.8cm较小，仍作为后续评审/实机外推风险，不放宽门槛。
`inflation-gradient-final-small-suite.oob6Bp`默认首例map2复跑147.698秒
SUCCEEDED但dynamic_collision=true，minimum=-.045138m，严格失败，后续
自动停止。源码/runtime/input校验、33项评分重放和TF通过，确认不是
评分采集故障。最差actor1，GT=76.364；此前2秒scan从.376降至.258m，
机器人仍以.1m/s前进，碰撞前才反向；不能将一次/两次成功视为可靠。
撤回衰减4及注释，构建后恢复原Arena3通过版本源码/runtime身份，保留
该版本BARN22停滞失败。两个原生代价候选均不能同时满足动态/静态小集。
当前仍等待是否允许时间相关MPPI评分扩展，不先实现，未跑全量或提交；
最终review通过条件未满足，Goal保持未完成。
用户批准时间相关MPPI critic，并要求先补review。新独立
`20261101-current-stack-prerepair-review.md`可提交0：D1小集缺失/失败、D4
smoke main CC27；D2/D3/D5/D6通过，46UT、原生测试、安装接口均通过。
主agent `20261101-current-stack-ponytail-review.md`提出未来占据丢时间、
SetEntityState失败继续运行、smoke职责混杂。评审均已完成但不是通过。
保留两份原报告。先将smoke速度/预测验证独立，包含tests的复杂度预检
无超限；障碍移动失败报错退出，不让只读采集掩盖运动协议失败，回归通过。
新增原生TimedObstacleCritic及pluginlib注册，不替换MPPI或引入框架。
输入仅LiDAR运动表面点xy/vx/vy和原始时间戳；使用实际Nav2矩形外包络，
在robot pose stamp+(j+1)*model_dt比较同刻恒速移动表面，原始云/pose过期
.5秒或非法输入使优化失败。现有真实LiDAR硬检查/安全监控/速度不变。
仅此候选关闭静态未来扫掠层，并恢复CostCritic常规253膨胀分级，动态
风险由独立时间critic评分；未放宽碰撞/定位/150秒评分。新输入完整归档。
46UT、构建、包括tests的新函数复杂度通过；原生4 tests包括实际plugin
加载、提前穿过与停留被侧向接近、空/陈旧/非法云、时间补偿、车体旋转。
安装测试初次提前退出等待第三路输出，已修等待条件并重测通过。
接着先运行map2/BARN22，不混用旧源码成功，不执行全量；新仿真未有结论。
安装接口测试还改用真实初始 IMU=(0,0,0)，验证 base=(0.03,0,-0.36)
及派生速度、扫描，避免只用初始 IMU z=0.36 的假设错过负高度问题。
未采用未验证参数，不混用不同候选的成功，不扩展识别/预测模块。
此前“需要先加动态预测”的判断过早，
目前先用现有原生 Nav2 验证此实际接口修复，不扩展新模块。

后续继续审查现有传感器链，发现投影器直接使用跨 100 ms 的滚动原始
点云，仅做固定外参变换，没有补偿 ego motion。官方 FAST-LIO 已发布
`/cloud_registered_body`：源码 `publish_frame_body` 将 `feats_undistort`
转换到 IMU body，并使用 lidar_end_time。当前配置 LiDAR/IMU 外参为
identity，因此与现有桥接的 IMU→base 变换一致。
已将导航投影器默认输入改为该已有输出、time_increment=0；静态投影
诊断显式继续订阅原始 `/lidar_points`。安装接口测试和 34 项 UT 通过。
`dwb-deskew-input-repair.73qZSj` 在 1.938 秒 FAILED：大点云 best-effort
接收有丢帧，扫描最大间隔 1.04 秒，安全监控拒绝陈旧输入。未放宽门槛。
默认 body cloud 输入改为 reliable depth=1，匹配官方可靠发布；显式
原始输入仍使用传感器 QoS。接口测试检查实际订阅端 reliability。
`dwb-deskew-qos-repair.6vHF5M` 重跑同一 map2，现场三秒接收检查得到
最大扫描时间间隔 0.10 秒、平均到达延迟 0.019321 秒、最大 0.028083 秒，
TF 审计通过。最终结果 102.102 秒 SUCCEEDED，定位和碰撞评分有效，
但 dynamic_collision=true，最小间隙 -0.245191 m，严格失败。整段采集
1009 帧扫描，最大时间间隔 0.230000 秒；source/runtime/input identity
校验及归档源码/观测/世界的 33 项指标离线重放通过，保留碰撞失败。
保留已验证的去 ego-motion 畸变及 reliable 接收修复，不能将其称为
动态避障已完成；未扩展运行 map5/BARN 或全量。
实机仍需测量 IMU/base 外参，不能沿用模拟 identity 假设。

map2 / obs5 / seed0 在 110.16 秒到达，但 scoring_valid=true、collision=true，
最小动态间隙 -0.3 m，严格失败。碰撞前约三秒扫描已看到 0.77 m 障碍，
随后扫描降至 0.24 m，控制器仍在持续前进；不存在这段障碍完全漏检。
因此没有继续 map5。新的 `dwb-clearance-repair.a234Tc` 候选仅将
ObstacleFootprint.scale 从 0.1 提高至 2.0；结果 101.864 秒 SUCCEEDED，
但 collision=true、最小间隙 -0.286750 m，严格失败，因此不保留这一调参。
之后读取 Nav2 原实现确认默认 sum_scores=false 只保留预测末点软代价，
虽然中间各点仍做硬碰撞检查，软代价不能体现整段临近障碍。
`dwb-trajectory-cost-repair.VbSN3f` 恢复 scale=0.1，启用内置整段累计和
固定 0.05 秒采样（所有速度候选同样点数）；结果 102.952 秒 SUCCEEDED，
但 collision=true、最小间隙 -0.250506 m，仍严格失败。
`dwb-trajectory-clearance-repair.EVzFgZ` 保持整段累计，将 scale 调至 0.5；
结果 101.728 秒 SUCCEEDED，但 collision=true、最小间隙 -0.277842 m。
三个调参候选均未通过，所以已撤回这些参数，恢复仓库严格通过时的
DWB 配置，保留修复后的 reliable `/odom`、底盘、采集和证据保护。
场景、actor 路径、速度、时间协议和评分始终未改变。
仓库成功记录属于 scale=0.1 的历史候选，不能与新候选拼成通过的小集。

下一步需选择更保守的纯 Nav2 安全策略，或增加不依赖识别模型的
LiDAR 障碍运动估计/短时预测。现有结果不支持继续将静态代价调参
当作已完成的动态避障修复；后一方向涉及新增模块，需明确方案选择。
map2 修复通过后才继续 map5 / obs5 / seed1。随后仍需同一冻结源码下的 BARN Quick-3
建图和导航，不混用历史版本的成功样例。最终独立 delivery-review 和
ponytail-review 尚未重新签发。

未验证全量与实机。JT16 通道配置、外参、安装支架和轮端动力学仍需
实测；零摩擦盒和水平仿真安装不能直接作为实机参数。

## 时间相关 MPPI 插件及采集修复续记

新独立评审：`20261101-timed-plugin-review.md`。D1 BLOCK、D2 PASS、D3 PASS、
D4 NITS、D5 PASS、D6 PASS，可提交0。未发现已证实的 temporal critic 逻辑
BLOCK；阻塞是同版双 Quick3 的可信成功证据未齐。建议的严格退出码已修：
run_nav2_goal 与 Arena/BARN 单例 shell 均传播严格评分失败，不只看 action。

新 native `TimedObstacleCritic` 使用实测 LiDAR 表面点及速度，在 pose/scan
时间差 + 每个 rollout 步时间上评分。保留真实 scan/CostCritic/Collision
Monitor；当前 MPPI 配置关闭静态未来扫掠层。46项原有UT、新critic3项
原生测试及旧prediction layer1项测试通过，installed ROS smoke已通过；
新增采集心跳/退出码回归后Python测试为48项。

| 候选 | map2 NAV150 结果 | 处置 |
|---|---|---|
| `timed-mppi-pilot.5H9WXx` | action SUCCEEDED107.388s；actor移动服务失败，评分时间缺口 | strict失败，保留 |
| `timed-mppi-service-diagnostic` | action SUCCEEDED135.948s；捕获返回码4及等待Gazebo状态超时 | strict失败，保留 |
| `timed-mppi-polled-state-pilot` | PROCESS_FAILURE46.57s；同一服务故障 | 新观测心跳及时取消，exit1 |
| `timed-mppi-native-pose-pilot`、`timed-mppi-native-blocking-pilot`、`timed-mppi-native-clock-pilot` | 原生接口启动/确认失败 | 不计成功，不扩评测 |
| `timed-mppi-native-ready-pilot` | SUCCEEDED139.452s；定位及评分有效、无input failure；collision=true，min−0.180315852m | strict失败，exit1 |

官方接口核对使用 ros_gz jazzy 的 `gazebo_proxy.cpp`、`set_entity_state.cpp`
和 gz-sim8 UserCommands 源码。ROS状态服务每次等待完整世界状态更新，
本机日志明确复现等待超时。原生批量pose服务已在隔离仿真验证类型及确认，
现在先等待发现/仿真时钟，再按原0.2s节拍命令；仍以0.1s独立实际Get观测
做评分，不以命令位置充当GT，不重试失败命令。没有改路径/速度/种子/评分。
有效native-ready候选的五个actor实测与原场景解析路径最大偏差分别为
0.022404/0.026005/0.029605/0.033205/0.036805m，观测最大时间差均小于0.2s。
source/runtime/input身份核对通过，TF审计PASS。

有效碰撞回放：actor1在仿真52.054s最小间隙−0.142116m，actor4在131.172s
最小间隙−0.180316m；两者此前已有LiDAR运动估计，但接近时出现短暂跟踪
丢失并仍有前进指令。先只提高时间风险权重100→1000进行同例对照，失败
则继续修跟踪连续性。未跑后续小集、全量或实机；不能声称插件交付完成。

后续：weight1000的 `timed-mppi-weight-pilot` 严格通过135.950s、min0.131662m；
相同源码 `timed-mppi-weight-repeat` 在141.612s到达但collision=true、
min−0.164414m，未进入BARN/剩余小集。仅调权重不能证明稳定成功。
随后给每个已确认track保留原确认时间，短暂失去确认时仅向当前scan时间
外推最多0.5s，不刷新确认时间、不无限延续遮挡。旧静态未来扫掠输出仍
按即时确认产生；延续只供时间critic的moving surfaces使用。原生Gazebo
transport/msgs及gzserver运行库现在一并记录SHA256。

`timed-mppi-motion-memory-pilot`：严格SUCCEEDED134.888s，零碰撞、
scoring/localization均有效、input_failure=null、min0.307222m，目标误差
0.069786m。NPZ离线重评分和归档source哈希核对通过。
独立 `20261101-motion-memory-review.md`：D1 BLOCK（最终六例未齐）、
D4 BLOCK（BARN缺失mapping分支exit0），其余PASS；未发现确认时间延续
逻辑错误。BARN该分支已改exit1，并增加不启动仿真的真实shell前置回归，
51 UT通过。此修复使pilot不再是当前完整源码身份，因此仅作诊断证据；
`timed-mppi-memory-repeat` 用新身份复跑map2，最终同版双小集仍待验证。

### 冻结版双 Quick3 实测结果

`integration/results/v0/timed-mppi-memory-repeat/`：不再改源码/运行库后逐例
采集，Arena与BARN各严格3/3，零碰撞、零input failure，定位和评分均有效。
六次导航和三次重新建图共享77项source、33项runtime SHA256。六例导航
NPZ离线重评分、归档source/input及当前身份核对、TF单发布者审计均PASS；
三例mapping的PCD/栅格/bundle SHA256和input-diagnostics NPZ均核对通过。

| 小集样例 | 导航秒 | 最小间隙m | 目标误差m | 严格成功 |
|---|---:|---:|---:|---|
| Arena map2 obs5 seed0 | 131.092 | 0.249985 | 0.053525 | 是 |
| Arena warehouse obs5 seed1 | 105.154 | 0.317865 | 0.086706 | 是 |
| Arena map5 obs5 seed1 | 78.876 | 1.564284 | 0.062514 | 是 |
| BARN22 seed0 | 88.414 | 0.169808 | 0.151130 | 是 |
| BARN55 seed0 | 65.334 | 0.139697 | 0.146170 | 是 |
| BARN77 seed0 | 47.924 | 0.502912 | 0.109507 | 是 |

| 新建图 | 规定路径点 | ATE RMSE m | occupied F1 | occupied IoU |
|---|---:|---:|---:|---:|
| BARN22 | 14/14 | 0.059103 | 0.884219 | 0.792466 |
| BARN55 | 19/19 | 0.073412 | 0.940545 | 0.887763 |
| BARN77 | 10/10 | 0.044855 | 0.909903 | 0.834699 |

Arena汇总 `arena/arena-summary.json`：成功率1、碰撞率0、unscorable0；
BARN `quick-summary.json`：success_rate1、promotion_gate_pass=true。
与上个soft-prediction同版完整候选相比，Arena3/3保持，BARN22从150s超时
变为88.414s严格成功，BARN55/77补齐。不是每项指标都单调改善：map2时间
由129.718s变为131.092s；不能声称所有指标提升或全量性能提升。
51 UT和4个原生测试重新执行通过，安装接口smoke及复杂度检查再次执行。
最终独立delivery/Ponytail review以完整冻结版为范围；本节不是评审签发。

未启动全量、未做实机验收、未commit/push。BARN规定路径采集不等于自主
探索；0.5s常速延续不证明遮挡/转弯/密集人群安全，仿真安装仍需实机标定。

### 删除过时 smoke 入口后的复跑失败与单例诊断

历史独立评审 `20261101-final-temporal-small-suite-review.md` 为 D4 BLOCK，
不是最终 PASS。删除无生产调用且缺失严格评分几何的旧 smoke 脚本后，
source 身份变化，旧 6/6 不再作为当前源码验收证据。
`timed-mppi-clean-final`：map2 严格成功 140.638s、间隙 0.308341m；
warehouse FAILED 119.984s、动态/静态碰撞、最小间隙 -0.247945m。
严格失败立即停止批次，未启动 map5 或 BARN。

离线逐帧定位首次动态碰撞为绝对仿真时间 113.220s（actor3），首次静态
碰撞为 118.422s。110s 时 AMCL 误差约 0.099m；末尾 0.635m 漂移发生
在碰撞/恢复之后。108–113s 存在持续逼近的 LiDAR 运动轨迹；112s 左右
控制命令转为旋转/零速度，之后 MPPI 退出、恢复停住，障碍撞入机器人。
并非只由 118s 后轨迹消失解释。日志显示多次控制频率低于 15Hz。

本轮先保留完全相同的评分函数，将每个 pose 的逐表面 hypot 改为最小
平方间距后一次 sqrt；增加多表面旋转轨迹与原公式一致的原生回归。
同时为 temporal critic 拒绝输入添加节流诊断，区别输入时效失败和其他
MPPI 无可行轨迹。尚未证明计算开销是碰撞根因；只运行 warehouse 新单例。

### 距离计算优化后的单例重复及小集进展（中间证据）

新原生测试先抓到 float 平方阈值导致空云产生极小非零风险；已将乘法
提升到 double，空云精确零分恢复。参考公式测试使用实际 costmap 带
padding 的 footprint，不以未加 padding 的评分 footprint 替代。
构建 PASS，原生 5 项 PASS，当时 Python 51 项 PASS。
最初提前启动的 `timed-mppi-distance-pilot` 已中止并保留，不计有效结果。

`timed-mppi-distance-retest` warehouse 单例严格成功 102.516s，最小动态
间隙 0.333714m，目标误差 0.097692m。逐例日志低于12Hz控制循环记录由
失败版187条降至1条；不是独立性能 benchmark，也不证明所有碰撞根因。
同版 `timed-mppi-distance-repeat` 的第二次 warehouse 和剩余 Arena：

| 样例 | 秒 | 最小动态间隙m | 严格成功 |
|---|---:|---:|---|
| warehouse obs5 seed1 | 105.272 | 0.187379 | 是 |
| map2 obs5 seed0 | 135.588 | 0.216330 | 是 |
| map5 obs5 seed1 | 78.968 | 1.525355 | 是 |

Arena汇总严格3/3，零碰撞、零input failure、定位/评分全部有效。
三例 NPZ 离线重评分、当前/归档源码/输入哈希和 TF 单发布者审计一致；
新独立审查代理也独立重评分核对通过，76 source /33 runtime。
BARN22新建图 COMPLETE14/14，ATE0.061012m；导航严格成功148.46s，
最小间隙0.197068m，零碰撞/input failure、定位有效。距NAV150仅1.54s，
不能宣称耗时或稳定性余量改善。BARN55/77未启动。

独立 review 随后确认建图入口错误：follow_barn_path超时exit0；批量
建图对已有不完整结果直接continue。停止后续批次并在无仿真进程后修复：
仅COMPLETE返回0；旧不完整bundle失败且保留历史；新建图失败立即停止；
缺失/空保存PCD返回1。补充退出表达式、真实无ROS子进程拒绝续跑以及
缺失/空PCD回归，现Python54项PASS，bash语法及lizard检查PASS。

这些入口修复再次改变source身份。上述Arena3/3与BARN22仅是中间算法
证据，不是当前源码的同版双Quick3验收。完整当前身份六例、BARN重新
建图和最终独立delivery/Ponytail PASS仍未完成；无全量、无实机、无提交。

### 当前源码补齐同版小集（重新评审证据）

本轮未再修改算法/传输配置/评分源码。构建、Python54项、native5项、
Shell语法和Lizard通过后冻结采集。`timed-mppi-current-gate` warehouse
严格成功102.538s，但map2在导航前出现PROCESS_FAILURE：MPPI配置成功
后`/controller_server/change_state` DDS应答超时，bt_navigator未active。
严格停止批次，无map5/BARN。该失败保留，不宣称全部启动尝试成功。

在新目录`timed-mppi-startup-pilot`用相同默认源码/参数/RMW单独确认
map2启动，未复现服务故障且严格导航成功。未复制旧输出，在该目录
继续补齐warehouse/map5和BARN22/55/77新建图及NAV150，逐例失败即停。

| 当前同版导航 | 秒 | 最小间隙m | 目标误差m | 严格成功 |
|---|---:|---:|---:|---|
| Arena warehouse obs5 seed1 | 113.612 | 0.187625 | 0.016352 | 是 |
| Arena map2 obs5 seed0 | 143.438 | 0.163012 | 0.091272 | 是 |
| Arena map5 obs5 seed1 | 79.258 | 1.525463 | 0.058723 | 是 |
| BARN22 seed0 | 149.304 | 0.182592 | 0.241547 | 是 |
| BARN55 seed0 | 64.976 | 0.122301 | 0.124103 | 是 |
| BARN77 seed0 | 41.800 | 0.570634 | 0.091961 | 是 |

| 当前新建图 | 路径点 | ATE RMSE m | occupied F1 | occupied IoU |
|---|---:|---:|---:|---:|
| BARN22 | 14/14 | 0.054993 | 0.886539 | 0.796202 |
| BARN55 | 19/19 | 0.116363 | 0.946719 | 0.898829 |
| BARN77 | 10/10 | 0.038029 | 0.895281 | 0.810415 |

Arena与BARN汇总各严格3/3，零碰撞/input failure、定位/评分均有效；
quick-summary promotion_gate_pass=true。六例NPZ重评分每项与JSON一致。
全部九次运行当前/归档source、runtime、input哈希一致（76/33），TF
审计PASS，三套5文件map-bundle哈希及轨迹/IMU诊断NPZ核对通过。
采集结束后Python54项与installed ROS smoke再次PASS，无遗留仿真进程。

范围仍仅小集：BARN22仅剩0.696s超时余量，存在历史DDS启动失败。
耗时/ATE并非逐项改善；前released V0不存在，不声称通过全量非回归。
0.5s常速运动记忆、shim/recovery绕过时间评分及实机标定限制仍保留。
新独立评审见`20261101-current-gate-rereview.md`；历史BLOCK不改写。
未跑完整集，未commit/push，未实机验收。
