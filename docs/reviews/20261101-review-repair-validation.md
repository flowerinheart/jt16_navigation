# Review 修复与验证记录

## 状态

代码层面的定位链、评分器、地图转换、安装和运行脚本修复已实施。
**尚未通过导航小集，不能认定 V0 完成，也没有启动全量评测。**

## 实施内容

- FAST-LIO 桥接按仿真外参将 IMU 位姿转换为 base_link，直接发布 /odom 和 odom→base_link；AMCL 唯一发布 map→odom。
- 删除不再使用的 GT 轮速适配脚本和 EKF 配置，启动脚本不再启动这条旁路。
- 导航检查器使用仿真时钟，并在局部里程计过期或 AMCL 协方差无效时取消任务。
- AMCL 样本带时间戳和协方差，消息收到率与有效定位成功率分开；Nav2 激活耗时不再冒充定位初始化耗时。
- Arena 障碍物移动请求结果检查后再读取实际状态，保存绝对时间、ID、位置和半径；缺数据/时间间隙标记不可评分。
- Arena 同时计入静态墙和动态障碍；最短路径禁止穿越阻塞角点，检查起终点。
- 地图膨胀不再回绕；轨迹只能将车体范围内 unknown 标成 free，不能覆盖 occupied；未观测区域保持 unknown。
- 在线投影与候选 pitch 参数一致，配置从 ROS package share 加载；写入程序创建父目录。
- Arena 批处理先生成公共模型，workers 不再重复写共享模型；独立单例仍可生成模型。
- CPU 统计累计各进程增量，不因子进程退出扣除历史耗时。
- 新候选归档执行源码、安装参数和哈希，检查恢复运行的身份一致性；旧结果不覆盖。
- Quick-3 检查固定成员、时间限制、schema 和碰撞证据；短 smoke 不能冒充150秒冻结任务。
- TF 审计读取 /tf 和 /tf_static，验证唯一发布者及必需边。Jazzy Python MessageInfo 缺 publisher_gid，因此增加一个小型 C++ ROS 探针读取实际 GID，Python 继续输出原审计格式。
- 正式审计检查非空地图、哈希、指标、运行身份和实测标定要求；生成场景加入 gitignore。

## 已执行验证

1. `PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -v`：13 项测试全部通过。
2. 全部 owned shell 脚本 `bash -n` 与 Python AST 解析通过。
3. `bash scripts/build.sh scaffold`：jt16_bringup 构建通过，包含 TF C++ 探针。
4. 在临时目录构建普通非 symlink 安装，启动并发送合成 ROS 输入：外参转换、/odom、TF、/scan 输出通过。
5. `integration/tests/ros_interface_smoke.py`：安装后的接口测试通过。
6. 实际 ROS 发布者测试：正确 TF 所有权可识别；/tf_static 上第二个发布者会被识别为重复。

## 短仿真记录

仅运行 BARN world22/seed0 的短 smoke，timeout=45秒，复用历史地图进行启动链诊断。
未重新生成本次修复后的地图，也未运行冻结 Quick-3/Arena 小集，因此这不是版本成功率评测。

- 首次：`integration/results/v0/review-repair-smoke.oPE0wu/barn22/`。
  时钟/输入检查立即取消；TF Python API 无法识别身份。记录保留，随后修复时钟配置与审计实现。
- 第二次：`integration/results/v0/review-repair-smoke.Xslswy/barn22/`。
  TF 审计 PASS，AMCL 和 LIO bridge 分别拥有 map→odom、odom→base_link，无重复边。
  实际行驶路径4.710m，评分未碰撞，最小几何间隙0.00375m。
  39.78秒时 AMCL 协方差无效，检查器取消任务，结果 PROCESS_FAILURE。
  最终目标距离5.391m，定位成功=false；最终 AMCL 误差2.784m，LIO误差3.597m。
  这暴露了恢复真实定位链之后的前端/定位质量问题，不能通过恢复GT旁路或放宽阈值掩盖。

两次 smoke 保留各自当时的源码快照。其后完善了评分schema、冻结timeout和身份记录，不回写旧记录。
历史 `final-dwb-r1` 原始证据未修改。删除的历史 GT adapter 可从 smoke 的执行源码快照恢复。

## 下一步与剩余验收

- 先定位 FAST-LIO 漂移原因：同步检查原始/适配 LiDAR 与 IMU 时间、运动模型、外参和匹配退化，不先调 Nav2 参数掩盖。
- 用修复后的转换器生成新候选地图，运行冻结 Quick-3；要求3/3物理成功、零碰撞、零进程失败。
- 然后运行固定 Arena 小集，验证实际障碍观测与评分重放；通过之前不启动全量。
- 硬件外参、实测报告和真实 JT16 通道校正仍缺失。本轮没有生成虚假的 hardware-calibration 文件，也没有豁免 V0 的硬件验收。
- 全量及实机未跑，新的导航成功率提升尚未证明。
