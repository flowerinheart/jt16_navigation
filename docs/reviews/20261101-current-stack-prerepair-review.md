# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **0** |
| **整体等级** | BLOCK |
| 一句话 | Arena严格3/3，BARN失败且缺两例，终态未达成 |
| D1 交付 | BLOCK |
| D2 验证 | PASS |
| D3 卫生 | PASS |
| D4 质量 | BLOCK |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | C1,C2 |
| 代码修改 | 必须 1 处；建议 0 处（见「代码修改建议」） |
| 评论数 | BLOCK=2；NITS=0 |
| 下一步 | 先处理挡住提交的 C1,C2；修复后冻结新候选，完整复验两套小集 |
| Goal complete | 0 |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`

范围: uncommitted；未跟踪 owned source 全部按新代码评审，包括 src/scripts/integration/scripts/tests/config/docs；空 diff 不代表无代码。

终态: `冻结 Arena Quick3 和 BARN Quick3 在同一源码及 runtime 下各3/3可信严格导航成功，零碰撞/进程失败，完整建图、传感、评分和身份归档。`

需求: `plan.md` 4.0；这是开发前现状评审。范围外: 全量评测、实机标定验收、commit/push、GT导航、场景/种子/速度/评分放宽、识别模型。轻量 LiDAR 运动估计/预测 costmap 已获授权；未写的时间相关 MPPI critic 不参与评审。

未执行: 未 commit、未 push、未改代码/配置/候选结果、未重跑仿真、未安装工具；仅写本报告，指定构建/测试更新了被忽略的 build/install/log。

## 六维打分

| 维 | 等级 | 命中的门槛（抄上面定义，勿空话） | 证据（路径/命令/exit/符号） |
|---|---|---|---|
| D1 交付 | BLOCK | 终态未发生；必交产物 missing / 达不到调用单通过标准 | C1：指定根 BARN22 TIMEOUT、55/77全部缺失、quick-summary缺失；Arena三例离线重评分严格通过 |
| D2 验证 | PASS | 绿、覆盖新行为、golden未放宽 | 指定46个 unittest、scaffold、原生 prediction_layer_test、安装ROS接口测试全部exit0；覆盖估速/静态墙误检/过期清除/真实障碍成本保护/评分终态/身份篡改/安装预测输出 |
| D3 卫生 | PASS | 每个路径已三分且无密钥、新依赖已pin、无残留垃圾 | 下表覆盖所有未跟踪顶层路径；密钥特征扫描无命中；dependencies三个清单固定commit；生成数据已有gitignore |
| D4 质量 | BLOCK | 新函数 CCN≥15 | C2：`integration/tests/ros_interface_smoke.py:40 main` lizard CC27/nloc60，radon --no-assert CC22；历史抽断言后仍超限 |
| D5 设计 | PASS | 同层同目录同模式 | 对读 run_barn_navigation/run_arena_navigation、lio_odom_bridge/project_nav_scan：复用共享runner、geometry/metrics、ROS Node及安装入口；预测复用Nav2 Layer/InflationLayer |
| D6 平台 | PASS | 有，且Goal/终态写明要扩框架（现有扩展点） | `PredictionLayer : nav2_costmap_2d::Layer`及pluginlib导出服务已授权预测costmap；`BoundedNavigator`覆写既有生命周期等待；ROS Node/spin、CLI main、TF probe进程为导航集成；无新ABC/Protocol/Base/registry/hook框架 |

本轮实际验证命令和结果：

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -v
# exit0，46 tests OK
bash scripts/build.sh scaffold
# exit0，1 package finished
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --base-paths src --packages-select jt16_bringup --event-handlers console_direct+ && colcon test-result --test-result-base build/jt16_bringup/test_results --verbose'
# exit0，1 test，0 errors/failures/skipped
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ROS_DOMAIN_ID=96 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout -k 3s 25s /usr/bin/python3 integration/tests/ros_interface_smoke.py'
# exit0，PASS installed bridge/TF/scan and LiDAR motion forecast interfaces
lizard -T cyclomatic_complexity=15 -T nloc=80 -T parameter_count=8 integration/scripts src/jt16_bringup/src integration/tests scripts
radon cc -s -n C --no-assert integration/scripts integration/tests scripts
radon mi -s -n B integration/scripts integration/tests scripts
```

质量工具已安装并执行；jscpd未安装，not run。lizard仅报 smoke main 超限；radon MI 显示 runner 17.09、test_regressions 2.66，未据此编造旧版MI下降。原生测试作为不受lizard解析的C++测试源也已人工阅读。

必交根均指 `integration/results/v0/lidar-soft-prediction-final-small-suite.aYJeYc/`。逐一检查明确枚举文件，没有借用其他候选：

| 成员 | 必交文件检查 | 严格结果 |
|---|---|---|
| `arena/map2_obs05/seed_00/` | nav2-goal、两NPZ、provenance、TF audit均非空 | SUCCEEDED；129.718s；距离0.054309m；strict flags通过 |
| `arena/small_warehouse_obs05/seed_01/` | 同上均非空 | SUCCEEDED；120.224s；距离0.095376m；strict flags通过 |
| `arena/map5_obs05/seed_01/` | 同上均非空 | SUCCEEDED；79.812s；距离0.094278m；strict flags通过 |
| `arena/arena-summary.json` | 2703 bytes；精确三个冻结目录，无额外nav2-goal成员 | runs_scored=3、SUCCEEDED=3、success_rate=1、collision_rate=0、unscorable=0 |
| `barn/world_022/mapping/` | mapping-result、map.pcd、nav-map/map.yaml、map.pgm、map-metrics、map-bundle-metadata、provenance均非空 | COMPLETE，14/14固定mapping waypoints；ATE0.057381m；PCD7885799点；五项bundle SHA256全部相符 |
| `barn/world_022/navigation/seed_00/` | nav2-goal、两NPZ、provenance、TF audit均非空 | TIMEOUT；150.008s；距离6.110148m；collision=false，scoring_valid=true，localization_success=false，scored_success=false |
| `barn/world_055/mapping/`、`barn/world_077/mapping/` | 各7项明确枚举文件全部missing | 无COMPLETE建图证据 |
| `barn/world_055/navigation/seed_00/`、`barn/world_077/navigation/seed_00/` | 各5项明确枚举文件全部missing | 无导航/传感/身份/TF证据 |
| `quick-summary.json` | missing | 无完整Quick3 promotion_gate_pass=true |

Arena三例均记录NAV150，scored_success/scoring_valid/localization_success=true、collision=false、input_failure=null；以NPZ、provenance参数及当前navigation_metrics独立只读重评分，五项严格结果与记录一致。两NPZ均可打开，传感包含scans/odom/commands/prediction/track非空样本。三例73项sources和22项runtime hashes逐项与当前文件相符，相互一致；source归档和world/map等输入hash相符。BARN22 mapping和navigation的sources/runtime也与Arena一致。四次导航TF审计均pass，map→odom仅AMCL、odom→base_link仅LIO bridge，duplicate_edges为空。BARN22 map-metrics为实际网格测量：occupied F1=0.885821、IoU=0.795044、wall mean=0.005127m；mapping脚本执行固定路径，不是自主探索。

历史建议独立复核：GT轮速绕过问题在当前runner/bridge未复现；FAST-LIO→bridge→odom、AMCL→map correction已接入。Arena真实GetEntityState观测、绝对时间关联、静态墙碰撞和失败动作不能成功的逻辑已存在，本轮NPZ重评分通过。AMCL消息存在性已与定位成功区分，BARN22即明确定位不成功。Quick3 scorer已有精确成员、严格flag、NAV150及动作成功门槛。旧full-release audit仍以终态枚举统计全量记录，是诊断审计，不能充当本轮小集成功证明；本轮未依赖或运行它。历史实机外参问题属于本轮范围外，未追加硬件验收阻塞。生成场景已忽略。旧smoke复杂度建议尚未解决（C2）；适配器仍未拒绝零/倒退obstacle stamp，但当前Arena真实样本重评分有效，下游有时间gap检查，本轮未将未复现的边角问题提升为交付阻塞。

## Git 三分

| 路径 | 处置 | 理由 |
|---|---|---|
| `.gitignore`, `LICENSE`, `README.md`, `plan.md` | commit | 忽略规则、许可和本轮需求/说明 |
| `dependencies.reference.repos`, `dependencies.runtime.repos`, `dependencies.templates.repos` | commit | 固定上游身份 |
| `docs/**` | commit | 锁定说明和历史/本次评审；保留历史失败记录 |
| `scripts/**` | commit | 构建及依赖管理入口 |
| `src/**` | commit | bringup源码、头文件、plugin描述、CMake/package、参数和原生测试；先修blocker |
| `integration/.gitignore`, `integration/scripts/**`, `integration/tests/**`, `integration/config/**`, `integration/benchmarks/manifests/**` | commit | 仿真集成、评分、回归及冻结manifest；先修blocker |
| `external/**`, `build/**`, `install/**`, `log/**`, `results/**` | gitignore | 已忽略，下载/构建/测试生成数据 |
| `integration/archive/**`, `integration/build/**`, `integration/install/**`, `integration/log/**`, `integration/deps/**`, `integration/results/**`, `integration/models/**`, `integration/worlds/**`, `integration/benchmarks/generated/**`, `**/__pycache__/**`, `**/*.pyc` | gitignore | 已忽略；保持候选及失败产物，不删除、不覆盖 |

git status顶层未跟踪路径就是上述owned内容；diff/cached diff/name-only为空，仓库根与调用单一致。无待revert路径，未执行git add/restore等写操作。

## 代码修改建议

| ID | 必须/建议 | 文件:行 | 函数或符号 | 改法（一句） |
|---|---|---|---|---|
| C2 | 必须 | `integration/tests/ros_interface_smoke.py:40` | `main` | 抽出已存在的速度验证和运动预测验证，保留循环时限、断言及finally清理 |

### C2 补丁示意

- 目标: `integration/tests/ros_interface_smoke.py` `main`
- 动作: 抽取两个已有验证阶段；下面为速度阶段完整示意，运动阶段按同样方式原样移动现有start/deadline循环及两个断言到`check_motion_forecast`。不减断言，不扩新框架。

```diff
+def check_derived_velocity(node, publisher, message, poses):
+    before = len(poses)
+    deadline = time.monotonic() + 1.
+    while time.monotonic() < deadline and not any(
+            abs(p.twist.twist.linear.x) > .01 for p in poses[before:]):
+        message.header.stamp = node.get_clock().now().to_msg()
+        message.pose.pose.position.x = .1
+        publisher.publish(message)
+        rclpy.spin_once(node, timeout_sec=.05)
+    assert any(abs(p.twist.twist.linear.x) > .01 for p in poses[before:]), \
+        'bridge dropped derived velocity'
+
 def main():
     ...
-    before = len(poses)
-    # 原有速度验证循环和断言
+    check_derived_velocity(node, publisher, message, poses)
-    # 原有start/deadline运动循环、预测和速度断言
+    check_motion_forecast(node, publisher, clouds, message, fields, predictions, tracks)
     ...
     finally:
         node.destroy_node()
         rclpy.shutdown()
         stop_children(children)
```

`check_motion_forecast(node,publisher,clouds,message,fields,predictions,tracks)`只承接现有第83–94行阶段，不引入配置或新测试库。重新运行lizard/radon及指定安装ROS接口命令，确保main及抽出函数均低于CC15且预测/实测速度断言仍执行。

## Review comments

| ID | 维 | 等级 | 挡住提交 | 位置 | 标题 |
|---|---|---|---|---|---|
| C1 | D1 | BLOCK | 1 | `integration/results/v0/lidar-soft-prediction-final-small-suite.aYJeYc/quick-summary.json` | BARN小集失败且证据缺失 |
| C2 | D4 | BLOCK | 1 | `integration/tests/ros_interface_smoke.py:40` | 新smoke main仍超复杂度门槛 |

### C1. BARN小集失败且证据缺失

- 维: D1 交付
- 等级: BLOCK
- 挡住提交: 1
- 位置: `integration/results/v0/lidar-soft-prediction-final-small-suite.aYJeYc/quick-summary.json`
- 问题: 指定根只有Arena3/3，BARN22失败、55/77缺失，不能声称同版两套小集终态。
- 证据: BARN22 TIMEOUT/150.008s/6.110148m，scored_success=false且localization_success=false；55/77各12个枚举文件missing；quick-summary.json missing。
- 代码修改: 无
- 建议: 先诊断修复BARN22停滞/定位，再用新空候选根冻结最终source/runtime，完整记录三例BARN固定建图和六例严格导航；保留当前失败，完整生成Quick3汇总。修代码后旧Arena不得与新BARN拼接；不启动全量，不改NAV150/成员/种子/速度/评分。
- 处置: 补证据

### C2. 新smoke main仍超复杂度门槛

- 维: D4 质量
- 等级: BLOCK
- 挡住提交: 1
- 位置: `integration/tests/ros_interface_smoke.py:40`
- 问题: 历史check_outputs抽取已做，但新增预测阶段使main依然超过新函数CCN≥15门槛。
- 证据: 本轮lizard main CC27/nloc60；radon去assert仍CC22，不能以“只是测试断言”消除门槛。
- 代码修改: 见「代码修改建议」C2
- 建议: 仅抽已有速度/预测验证阶段，保留所有断言、超时和清理；重新运行质量工具及安装ROS smoke。
- 处置: 改代码

未检查全量评测、实机标定和未写critic，未重跑仿真；当前仿真成功不能证明JT16实机性能或真实机器人避障安全。
