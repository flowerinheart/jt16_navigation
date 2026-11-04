# V1 开发记录

开始：2026-11-03 00:37:43 CST。目标与放行标准见 [plan.md](plan.md)。
原计划期限：2026-11-03 12:37:43 CST；用户于11月3日暂停整个任务，11月4日通知继续。
暂停前累计执行约50分钟；暂停时间不作为开发执行时间。当前未达成 V1 放行。

## P0：输入和建图生命周期

V0 全量批次遗留 28 个建图 bridge 子进程，使用同一 ROS domain。
在原环境复跑277：FAST-LIO初始化并处理点云，但采集侧仍零条LIO轨迹；
路径完成后等待地图服务，未得到有效地图。证据：
`integration/results/v1/p0-baseline/world_277/mapping/`。

通用修改：建图子进程使用独立进程组并整体清理；复用导航的ROS domain隔离规则；
采集等待新鲜、有限的GT/LIO输入才移动，输入丢失立即停止并记录INPUT_FAILURE。
路径完成与地图有效分开：有效地图还要求配对轨迹指标、非空地图包、有限F1/IoU。
保存地图服务设有界等待；导航入口拒绝未完成或明确无效的地图。没有world/seed特判。

清理历史残留后的首次277新建图：443条LIO，ATE RMSE 0.0633m，地图包有效。
证据：`integration/results/v1/p0-lifecycle/world_277/mapping/`。
清理与domain隔离同时应用，暂不将DDS恢复归因于其中单独一项。

随后连续启动/关闭277→11→277，均生成有效地图：

| 顺序 | 世界 | LIO样本 | ATE RMSE m | F1 | IoU |
|---|---:|---:|---:|---:|---:|
| 1 | 277 | 457 | 0.0587 | 0.8842 | 0.7924 |
| 2 | 11 | 469 | 0.0763 | 0.9284 | 0.8663 |
| 3 | 277 | 465 | 0.0545 | 0.8991 | 0.8166 |

证据：`integration/results/v1/p0-sequence/{first,middle,last}/`。

## P0：未知地图区域的规划

聚合点云地图仅将实际走过的footprint标为free，其余未观测区保留unknown。
BARN11/55目标栅格为unknown，原NavFn禁止经过unknown，日志显示规划失败。
改为允许NavFn经过未知区，保留unknown栅格、实时障碍层及Collision Monitor。
未改变规划器、机器人几何、速度上限或150s导航时限。

| 样例 | V0全量 | 当前开发候选 |
|---|---|---|
| BARN11 seed1 | 16.504s失败 | 45.542s严格成功；零碰撞；最小间隙0.641m |
| BARN55 seed0 | 16.644s失败 | 76.070s严格成功；零碰撞 |

该单例复用V0地图以隔离导航参数影响；放行时仍须新建全部开发集地图。
证据：`integration/results/v1/p0-unknown/world_011/navigation/seed_01/`。

## 检查与待办

Python回归59项通过；原生5个测试用例通过；owned package构建通过。
完整Dev16、动态复跑、210次全量导航及30次建图尚未完成。

BARN177的Nav2激活恢复正常（2.248s），但仍150.054s超时，目标距离0.663m；
90～140s间命令几乎为零，60s进度等待后清理costmap/重置优化器才恢复移动。
进度检查等待恢复为已安装Nav2示例的10s，顺序实测如下，均零碰撞：

| 样例 | 新用时 s | 终态 |
|---|---:|---|
| BARN177 seed0 | 97.898 | 严格成功 |
| BARN99 seed0 | 69.830 | 严格成功 |
| BARN144 seed0 | 77.078 | 严格成功 |

证据：`integration/results/v1/p1-progress/`。

map2 obs5 seed0复验138.536s到达目标，输入过期未复现，但碰撞、间隙−0.130m，严格失败。
证据：`integration/results/v1/p0-arena/map2_obs05/seed_00/`。
继续验证动态碰撞和停车恢复。
V0 warehouse obs10首次接触发生在任务74.936s时，机器人几乎停止，仍有运动预测点；
不能仅靠修输入过期或停车保护解释该失败，需要核对预测与实际执行。

开发集统一入口：`/usr/bin/python3 integration/scripts/run_development_set.py --manifest integration/benchmarks/manifests/v1_dev16.json --output integration/results/v1/<new-candidate>`。
入口为每个BARN世界新建图，再执行混合清单全部导航；记录失败并继续，不覆盖历史结果。

warehouse obs10旧配置复验150.012s超时；末端一名障碍观测缺口0.274s，评分无效。
新增有界末端消息收集，任务终点和耗时冻结，评分间隔限制保持0.25s；不延长导航时间。
10s进度等待和末端收集复验：map2 obs5严格成功136.798s；warehouse obs10超时且碰撞，评分有效；map2 obs10因输入过期退出。

## 暂停与恢复

速度反馈实验将Collision Monitor已有SlowCircle上限通过原生`/speed_limit`反馈给MPPI，
读取同一份有效Nav2配置，避免控制器按未执行的速度预测到达时间。尚未证明该实验有效。
此前ComfyUI占满共享GPU，仿真实时因子降至约0.40；用户选择暂停整个V1任务，ComfyUI未被停止。
暂停清理时BARN144已经启动，该次中断结果不计入放行证据。

11月4日恢复检查：GPU空闲，无残留导航任务；最新源码重新构建；62项Python回归和ROS接口检查通过。
旧速度反馈批次map2 obs5输入过期，map2 obs10超时且碰撞，warehouse obs10输入过期且碰撞。
正在空闲环境重测受影响案例；完整Dev16和最终批次尚未开始。

恢复后的对照实验（同一场景/seed/150s评分，独立输出目录）：

| 配置 | 样例 | 终态 | 用时s | 最小间隙m | 终点距离m |
|---|---|---|---:|---:|---:|
| 速度反馈，预测余量0.35m | map2 obs5 | 严格成功 | 135.720 | 0.340 | — |
| 速度反馈，预测余量0.35m | warehouse obs10 | 超时、碰撞 | 150.192 | −0.179 | 7.588 |
| 速度反馈，预测余量0.50m | warehouse obs10 | 严格成功 | 134.874 | 0.486 | 0.100 |
| 速度反馈，预测余量0.50m | map2 obs10 | 超时、零碰撞 | 150.022 | 0.520 | 5.568 |
| 上项 + PathFollow偏移5→15 | map2 obs10 | 超时、零碰撞 | 150.046 | 0.518 | 4.625 |

证据：`integration/results/v1/resume-{speed-feedback,clearance,lookahead}/`。
以上均无输入失败。0.50m余量两个样例均未触发近距离限速，暂未证明速度反馈的独立收益。
map2 obs10执行速度约0.20m/s，低于0.35m/s上限；正在按已安装PathFollowCritic文档验证路径权重。
不修改评分、场景、导航时限或速度上限；不按样例ID分配运行配置。

PathFollow权重5→15后，map2 obs10平均速度0.262m/s，零碰撞，但150.028s时距目标0.146m，
仍未满足0.10m控制器容差，按失败保留。证据：`resume-path-weight/map2_obs10/seed_00/`。

删除未证明独立收益的速度反馈、专用CLI/配置读取与相关测试；运行时继续使用原有LiDAR运动预测。
当前候选PathFollow权重20、偏移15、时间预测余量0.50m，其他碰撞代价不变：

| 样例 | 终态 | 用时s | 最小间隙m |
|---|---|---:|---:|
| warehouse obs10 | 严格成功 | 132.586 | 待汇总 |
| BARN144 | 严格成功 | 62.590 | 0.155 |
| map2 obs10 | 超时 | 150.080 | 0.133 |

证据：`integration/results/v1/p2-candidate/`；BARN144仍是导航-only复用V0地图。
map2本轮89.8%命令在倒车，终点距离14.86m；Shim仅在初次路径朝向调整，避让后未恢复朝向。
正在验证原生持续朝向校正及其标准运动阈值，避免把终点角度容差用于运动路径朝向校正。

有效地图统计还排除了“指标有限但地图包不完整”的样本；61项回归通过。
只读重算V0仍27有效/3无效，ATE均值0.160712m、F1均值0.864664，未改写V0证据。

## Dev16 候选 A

持续Shim校正使map2产生306次控制周期延迟、71次TF外推错误，两个密集动态样例均失败，未采用。
纯MPPI曾使map2 obs10严格成功146.734s，但warehouse obs10失败，尚不能单例放行。
后续近终点推进配置的BARN244严格成功69.708s、间隙0.0667m；动态样例仍未稳定。

候选A使用通用控制参数：PathFollow权重15/偏移15、Goal权重10、PathAlign权重6，
时间避障余量0.50m；关闭每周期重新生成噪声。保留起步Shim，终点朝向由MPPI评分。
NavigateToPose恢复树沿用原生模板的重规划/清图/等待，移除未经过时间评分的Spin与BackUp。
实测Nav2加载了安装目录的新树；构建与61项Python回归通过。没有样例ID分支或评分改动。

统一入口正在执行`integration/results/v1/dev16-candidate-a/`，批次内不改配置、不覆盖结果。
十张地图已全部新建有效：平均ATE 0.0580m，最大0.0922m，最少448条配对样本。
导航正在按清单执行；完整16/16、动态复跑和全量放行仍待完成。

候选A最终11/16严格成功，1次碰撞，零输入/进程失败；五个失败全部保留：
map2 obs5到达但碰撞，warehouse obs10/map5 obs10/BARN244在终点附近超时，map2 obs10超时。
近终点两个Arena样例的地图坐标误差0.113/0.129m、朝向误差0.020/0.004rad，末端预测风险为0。
探索噪声预算缩放复验只有BARN244成功，动态样例退化，已删除该实验代码。
终点朝向评分恢复与0.10m目标检查器一致，避免在补位置时提前牵制朝向。

## 几何配准与候选 B

移动表面用参考局部法向估计平移，滑窗速度不再拟合可见质心；
关联允许可见面缩小，并保留增长/位移/残差边界。没有物体类别、场景或seed分支。
受控部分可见表面：真值速度(0.2,−0.1)，原质心法(0.164,0.151)，配准恢复真值。
新增部分可见运动、静止、轨迹连续性回归；64项Python回归和ROS接口检查通过。

| 复验样例 | 严格成功用时s | 最小间隙m |
|---|---:|---:|
| map2 obs5 | 92.220 | 0.398 |
| warehouse obs10 | 97.582 | 0.445 |
| map5 obs10 | 82.352 | 0.719 |
| BARN244 | 92.128 | 0.050 |

证据：`integration/results/v1/p3-registration/`，四例均零碰撞/输入失败。
BARN244导航-only复用候选A地图；这不是放行批次。
已启动`integration/results/v1/dev16-candidate-b/`，十世界再次新建图、全部16导航，批次内不改配置。

## 候选 B 进行中的诊断（11月4日）

10/10新地图有效；导航批次仍在运行，不提前计为放行。
warehouse obs5动态碰撞时，预测速度与局部GT拟合一致；靠近车体后可见扫描由48点缩为4点并消失，0.5s轨迹记忆随后到期。
warehouse obs10碰撞时预测仍正确，因此短时遮挡不是全部原因。
两次碰撞窗口中原始指令均约0.21m/s，执行/里程计仅0.10–0.12m/s；
圆形近距限速对后方移动障碍同样生效，使车体实际运动偏离MPPI避让预测。

外部参数对照只移除圆形近距限速，保留FootprintApproach车体轮廓/2s碰撞预测：
warehouse obs5到目标附近0.100m、最小间隙0.449m、零碰撞，但里程计一次0.60s间隔触发输入过期，仍是PROCESS_FAILURE，不能计成功。
证据：`integration/results/v1/p4-execution/warehouse5/`。没有放宽0.5s输入检查。
BARN144候选B中途FAILED：轨迹偏离全局路径并靠近通道边缘，重规划无路；没有实际碰撞。
正在独立验证PathAlign权重14，使用同一新建图身份。
遮挡2s保留/重关联草稿仅位于/tmp，尚未进入生产代码或候选B。

候选B最终：10/16严格成功，3例碰撞，1例里程计输入过期；十张有效新图。
六个失败：warehouse obs5成功但碰撞；warehouse obs10超时且碰撞；map5 obs10成功但碰撞；
map2 obs10无碰撞超时；BARN144中途重规划无路；BARN111里程计过期。
并行BARN144路径对齐对照也因里程计过期退出，未得到参数结论，后续全部串行。
原生Nav2 1.3.12的Source::getTransform在base_shift_correction开启时要求当前时刻TF，
记录中多次请求比最新LIO TF晚约0.02–0.03s，等待后停车。
已启动仅使用base_link扫描的原生配置对照，保留过期与FootprintApproach规则；待完成。

车体扫描串行对照：domain105在smoother生命周期服务响应超时，60s有界退出。
domain106正常，确认运行参数base_shift_correction=false，未再出现当前时刻TF外推停车。
warehouse obs5严格成功60.824s，最小间隙0.480m，输入失败为None；
同一B候选该例119.922s且动态碰撞。只是单例对照，不是放行。
证据：`integration/results/v1/p4-execution/warehouse5-local-domain106/`。
正在同配置串行检查warehouse obs10。

warehouse obs10车体扫描配置无碰撞/输入失败，但150.070s超时，目标距离8.842m；末端原始/执行/里程计速度一致。
加强PathAlign至14和PathAngle至8后141.652s到达，但途中碰撞，仍不合格。
碰撞前对象速度已被正确估计，实际指令持续倒车(-0.1m/s)并转向；暂不采用2s遮挡草稿来解释这一失败。
同一方向评分配置BARN144严格成功42.054s、间隙0.0634m，复用B新建图；原B中途重规划无路。
正在原生MPPI两次迭代的独立对照，检查动态局部优化和控制周期，不改变评分或速度上限。

## 全局软预测对照

原生两次MPPI迭代110.628s到达，但两次动态碰撞（最小间隙−0.236m），不采用。
启动审计确认TimedObstacleCritic实际权重1000/余量0.50m；同配置单次迭代又135.508s严格成功，
但最小间隙仅0.0087m，仍不稳定，不能挑这次通过算修复。
现复用已有PredictionLayer给全局规划软成本；局部仍仅用时间匹配critic。
增加按观测时间的odom→map TF转换，转换失败不使用错误坐标，旧预测仍0.5s过期并清除。
默认DWB全局层关闭，MPPI配置开启；已有原生预测成本上限252不变，不添加场景分支。
64项Python、6项C++测试（新增旋转/平移跨帧预测）与ROS接口检查通过；构建通过。
PathAlign14/PathAngle8保留，原生迭代次数仍1。warehouse obs10对照已启动，待完整评分。

全局软预测warehouse obs10严格成功129.376s，最小间隙0.3263m，零碰撞/输入失败。
启动前map TF未建的预测被拒绝，进入导航后没有转换错误，局部层关闭、全局层开启已由运行配置和日志确认。
正在同源码配置串行验证map2 obs10；不能单例放行。
证据：`integration/results/v1/p5-global-forecast/warehouse10/`。

map2 obs10同一配置严格成功97.000s，间隙0.3746m，零碰撞/输入失败。
两个密集动态样例都保留了正间隙；仍不能替代完整开发集与复跑放行。
已冻结源码/安装库并启动`integration/results/v1/dev16-candidate-c/`：重新建十张地图、全部16例串行导航，批次内不修改源码/配置。
2s遮挡延续草稿未采用；两次MPPI优化迭代也未采用。

对比工具`docs/v1/compare_results.py`只在临时目录重汇总有效地图，V0原始文件不变；
导航评分模块与V0评分几何的指纹相同。两个报告回归通过：重现V0全部五项目标计数；缺失记录不得达标。
绘图/回放入口`docs/v1/render_report.py`读取实际障碍ID/半径、配置footprint和有界观测时间；
完整批次未完成时拒绝生成完整报告图，Dev16回放要求源码身份及严格成功。尚未生成最终图/MP4。

## 候选 C 中途发现的静态问题

前三个Arena obs5严格通过，间隙0.486/0.518/0.816m；warehouse obs10也严格通过82.812s、间隙0.469m。
BARN77却在起点NavFn提取路径失败，日志明确“legal potential was found”；预测点和运动轨迹都为0，不是全局动态层堵路。
同一新建图按既有0.347m保守半径离线验证，可达路线10.497m，起点/目标障碍间距充分。
正在准备原生Smac2D对照（插件已安装），不改机器人几何/地图/目标/allow_unknown规则。
BARN11 seed1无碰撞超时，GT距目标0.477m、局部指令末端持续小速度转向；
准备验证PathAngle使用原生PathAlign的0.5m近目标区域，GoalAngle仍0.1m，实际目标检查器不变。
两项对照参数暂存/tmp，候选C保持不变并跑完所有样本。

候选C最终14/16严格通过，零碰撞、零输入/进程失败，10/10新地图有效；
全部5个Arena样本通过；仅BARN77的起点路径提取失败、BARN11 seed1的近目标超时未解决。
已结束C批次，开始两项串行单项对照，之后仍需全新的Dev16批次。

Smac2D单项对照：BARN77同一C新建图严格成功37.924s，间隙0.5532m，零输入失败；
仅全局插件替换，GoalAngle/PathAngle阈值仍C配置。插件二进制额外SHA256保存在planner-binary.json。
正在BARN11 seed1仅PathAngle近目标区域的单项对照，全球规划器仍NavFn。

BARN11 seed1单项对照严格成功35.456s、间隙0.4896m、目标距离0.0528m；仅PathAngle使用原生PathAlign 0.5m近目标区域，仍NavFn。
两项修复现已合入：全局Smac2D（allow_unknown/0.25m规划容差不变）；近目标只停止路径方向评分，GoalAngle/目标检查器仍0.1m。
二进制身份现在包含已执行Smac2D库，已有篡改检测回归覆盖。
构建、64项Python回归、ROS接口检查通过；C++代码相对前次6项通过未再改动。
已启动`integration/results/v1/dev16-candidate-d/`，重新建十张图，全部16导航串行，源码/安装库冻结。

## 候选 D 通过开发集

完整16/16严格通过；0碰撞、0输入/进程失败，10/10新图有效。
证据：`integration/results/v1/dev16-candidate-d/development-summary.json`。
已开始map2 obs5 seed0、warehouse obs10 seed0各3次串行独立复跑，全部结果保留。
源码/安装库仍冻结；复跑全部通过后才启动完整30/150/60批次。

## 复跑通过并启动全量冻结评估

map2 obs5 seed0、warehouse obs10 seed0各3次，6/6严格通过，0碰撞、0输入/进程失败。
证据：`integration/results/v1/repeated-d/repeat-summary.json`，每次独立目录/ROS域，保留全部结果。
已启动`integration/results/v1/full-benchmark-20261104/`：30张新图、150次BARN、60次Arena，全部串行、无自动重试、不中途修改源码/参数。
当前只有开发门槛与重复性门槛通过，五项全量性能目标尚待检验。

全量建图阶段已完成：30/30新地图有效，runner退出0。
导航中已出现两例AMCL激活失败（BARN11 seed3、BARN33 seed0）；
原始定位日志分别记录`/amcl/change_state`和`/map_server/change_state`响应发送超时，
生命周期管理器停在配置阶段。结果保留为PROCESS_FAILURE，不重试覆盖，批次继续冻结。

## 独立评审发现未知栅格合并缺陷，候选 E 重启验证

Ponytail独立评审发现全局预测在未知栅格(255)被`max`吞掉：Smac允许通过未知区，
但预测前后路线成本相同。采用Nav2原生最大合并语义：未知格接收预测成本，其余格取最大值，
不改变252软成本上限、真实致命障碍、输入新鲜度或物理/评分约束。
新增未知格回归验证插入、远处未知保持、致命格保护和过期恢复。
构建、64项Python/2项报告回归、7项C++测试、ROS接口检查通过。
候选D全量批次明确标为`ABORTED_FOR_REVIEW`，保留已采集原始记录，不作为最终成绩。
新候选`dev16-candidate-e`重新建十张地图并验证全部16例；通过后再进行六次复跑和新全量冻结批次。

候选E已完成：16/16严格成功、10/10新地图有效、0碰撞/输入/进程失败。
证据：`integration/results/v1/dev16-candidate-e/development-summary.json`。
独立Ponytail代码复审通过（[报告](../reviews/v1-ponytail-review.md)），性能放行仍待复跑与完整评估。
已开始`repeated-e`六次串行复跑，每次保留独立记录。

候选E复跑最终5/6严格成功；第三次warehouse obs10碰撞，最小动态间隙−0.2336m，
同时发生墙体碰撞，无输入失败。该候选未通过重复性门槛，未启动新全量批次。
动态首次接触仿真103.972s，墙体首次接触106.148s；之前在墙边转向，
执行速度被FootprintApproach连续压低，障碍折返后追上；定位误差约4–8cm。
全部六次结果保留于`repeated-e/repeat-summary.json`。
开始仅动态critic间隙0.50→0.75m的三次独立warehouse obs10对照，
临时参数及独立身份保存在`p7-turn-margin`；源码/默认配置暂不改动。
机器人几何、速度上限、2s碰撞投影、评分/期限均不变；单项对照不代表性能放行。

0.75m动态间隙三次warehouse对照全部严格成功、零碰撞/输入失败，用时116.246/132.036/140.160s，
实际最小间隙0.0388/0.0992/0.1188m；效率和实际间隙仍有风险，不能据此宣称稳定。
map2相邻对照obs5/obs10均严格成功95.912/107.790s、间隙0.6554/0.5401m。
已采用这一通用参数形成候选F，仅修改现有critic动态余量；不改变静态几何/物理/评分/碰撞保护。
候选F将重新完成Dev16与六次复跑，仍受总计12h主动工作时限约束。

候选F首例warehouse obs5 seed1在42.372s因预测过期退出，零碰撞、定位有效。
记录扫描54.97991667s，最近odom55.02325s，差43.33ms超过30ms配对条件；
扫描两侧odom54.74991666/55.02325s间隔0.2733s，有可插值的有效姿态。
该扫描被丢弃，使预测54.72991667→55.23741667s间隔0.5075s，触发既有0.5s过期保护。
节点Traceback发生在关闭阶段，不是这次预测中断的证据。
正在/tmp验证扫描时刻姿态插值；不放宽预测/扫描新鲜度，不在F冻结批次中修改源码。

候选F最终14/16严格成功、0碰撞、10/10有效地图。
第二个失败为BARN111 seed0：30.732s FAILED，多次MPPI全部轨迹不可行，停车后AMCL过期；
定位误差0.026m，零碰撞。最后AMCL姿态距离保存地图占用中心0.187m，矩形footprint覆盖占用格。
Nav2 1.3.12 `smac_planner_2d.cpp`明确以`use_radius=true`配置碰撞检查；
矩形的内切半径不足以保护四角。开始同一F地图的原生全局外接圆单项对照，
由已有矩形和默认padding推导0.322684m，局部矩形/实际机器人/评分保持不变。

外接圆同图对照FAILED 22.574s（无有效路径/Start occupied），不采用。
原生Lattice首例被Fast DDS的AMCL应答超时阻断；Cyclone DDS下同图对照：
原生Smac2D严格成功56.550s、间隙0.0549m；Lattice FAILED14.508s，无碰撞。
Lattice也不采用；这些实验未改变默认规划器，不能据此宣称BARN111所有新图稳定。

候选G仅新增两项通用修复：
扫描时刻姿态由相邻有效odom插值（间隔≤0.5s、最短yaw角），无外推，保留原30ms近邻条件；
三个采集/导航脚本默认采用已安装Cyclone DDS，可沿用原生RMW环境覆盖。
新增两个姿态插值回归；66项Python、构建、Cyclone ROS接口检查通过。
中间件动态库与ddsc身份已加入采集哈希；依赖声明仅已安装系统运行包，没有安装新依赖。
全局仍Smac2D，动态间隙0.75m，物理/地图/评分/新鲜度/2s碰撞保护不变。
已启动`dev16-candidate-g`重新建十张地图和验证16导航；源/安装库冻结。

候选G已完成：16/16严格成功，10/10有效新图，0碰撞/输入/进程失败。
warehouse obs5严格成功62.766s，最大预测时间戳间隔0.100s（F失败时0.5075s）；
BARN111同一候选新图严格成功110.524s。独立代码复审已通过新增姿态/中间件改动。
已开始`repeated-g`六次独立复跑，全部保留；通过后才启动新全量冻结批次。
Dev16选定三案例的地图/轨迹图和8倍速遥测回放已生成于`docs/v1/assets/`，
来源身份逐例验证，视频H264/1200×500/12fps/14.25s；不是全量性能验收证据或相机录像。

候选G复跑6/6严格成功，0碰撞/输入/进程失败（`repeated-g/repeat-summary.json`）。
已开始`full-benchmark-20261104-g`：30次新建图、150次BARN、60次Arena，串行、无自动重试。
采用统一Cyclone DDS，代码和安装库保持冻结；先前D/E/F与全部失败原始记录继续保留。
本轮仍遵守累计12h主动工作上限，约2026-11-04 11:56 CST；
完整批次若届时未完成则记录中断，不能计作V1放行或完整评估。

2026-11-04 10:44 CST冻结全量检查：30/30新地图有效，BARN60/62严格成功、零碰撞/PROCESS_FAILURE。
world111 seed3 FAILED28.280s，Smac2D连续no valid path found后action abort；最终距目标8.687m，定位平移误差0.028m。
seed4 TIMEOUT150.076s，最终距目标2.694m，定位平移误差2.838m、LIO误差2.625m；零碰撞、scoring_valid=true。
上述日志支持失败现象，尚未证明根因；保留原始结果，不修改冻结候选、不重跑覆盖。

2026-11-04 11:42 CST：BARN123/126严格成功，新增world255 seed0 FAILED16.516s。
Smac2D报Start occupied，随后BT halt follow_path结果读取失败并退出；定位有效，平移误差0.0366m、LIO0.0420m，
最终距目标5.3088m，最小障碍间隙0.0563m，零碰撞/输入失败、scoring_valid=true。
日志支持上述退出链路，未完成根因验证；原始结果保留，冻结配置不变。
独立delivery-review快照结论D1 BLOCK/可提交0，指定66+2 Python及3+4 C++通过，
D4/D5/D6 PASS；报告docs/reviews/v1-delivery-review.md。

2026-11-04 11:49 CST：BARN127/131严格成功，world266 seed0 TIMEOUT150.098s；
最终目标距离0.4415m、定位有效（平移误差0.1927m）、最小间隙0.0457m，无碰撞/输入失败。
日志有早期MPPI optimizer fail，终点未满足严格到达；尚未完成根因验证。

2026-11-04 11:51 CST终态：累计12h主动工作限制，11:50停止调度，当前world266 seed2正常写入结果，
11:51仅终止本次冻结评测所属进程组；验证无存活所属进程，未操作其他任务。
request.state=ABORTED_TIME_LIMIT，budget-stop.json保留停止时间、当前案例及所属进程组。
最终30/30有效地图；BARN129/133严格成功（2 FAILED、2 TIMEOUT），0碰撞/输入/PROCESS_FAILURE；
17 BARN和60 Arena未执行，不视为测得失败或成功。完整性false，V1未放行，不commit/tag/push。
刷新英文report、comparison、failures与身份/协议审计：196/273现有身份全部源码/运行库/归档输入匹配，
133个已有案例固定物理输入与V0匹配；未修改冻结实现或评分，保留全部原始批次。
交付评审C2生成媒体精确路径已加入.gitignore，媒体及哈希证据保留本地；C1完整评测证据仍未满足。
后续需另行授权超出累计时限的工作，才能完成缺失冻结评测并重新独立交付评审。

2026-11-04 12:50 CST 用户要求暂停并记录进度：本次补跑所属13进程组/36进程全部SIGSTOP，
调度、ROS、Gazebo及当前case停止；保留进程状态，未终止当前案例或操作其他任务。
BARN150/150记录、144严格成功、0碰撞、1 PROCESS_FAILURE、2 FAILED、3 TIMEOUT；
Arena10/60记录、10严格成功、0碰撞/PROCESS_FAILURE；当前warehouse obs10 seed0仍在执行状态被冻结。
50个Arena结果待完成（含当前冻结案例），超时单独复跑阶段尚未开始。
补跑协议保持：不重跑成功/明确失败，仅缺失案例+每个原TIMEOUT一次；原133例所有源结果哈希仍一致。
暂停状态/进程PID起始ticks/恢复方式在full-benchmark-20261104-g/orchestration/user-pause.json；
恢复前须核对PID身份，SIGCONT记录的所属组；墙钟看门狗在暂停期间仍计时，恢复时可能触发，必须诚实记录。
详情docs/v1/pause-checkpoint.md；等待用户恢复通知，不自行继续。
