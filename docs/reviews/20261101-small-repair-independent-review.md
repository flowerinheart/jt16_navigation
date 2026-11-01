# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **0** |
| **整体等级** | BLOCK |
| 一句话 | 小集仅成功1/3，Arena无验收证据，复杂度待修 |
| D1 交付 | BLOCK |
| D2 验证 | NITS |
| D3 卫生 | PASS |
| D4 质量 | BLOCK |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | C1,C3 |
| 代码修改 | 必须 1 处；建议 1 处（见「代码修改建议」） |
| 评论数 | BLOCK=2；NITS=2 |
| 下一步 | 先处理 C1–C3；不得启动全量或将 Goal 标 complete |
| Goal complete | 0 |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`

范围: uncommitted，所有未跟踪 owned source 按新代码评审；external/install/results 等生成数据不作为待提交源码。

终态: 固定 BARN Quick3 和 Arena Quick3 导航可信成功，不使用 GT 导航回退、不放宽门槛。

需求: `plan.md`；范围外: 全量 benchmark、硬件标定验收、commit/push。

未执行: 未 commit、未 push、未改代码；仅写本报告。评审时间点的在途采集不视为通过。

## 六维打分

| 维 | 等级 | 命中的门槛 | 证据 |
|---|---|---|---|
| D1 交付 | BLOCK | 终态未发生；必交产物达不到通过标准 | `rigid-planar-candidate.I21Ms4/quick-summary.json`: successes=1/3、process_failure_rate=1/3、promotion_gate_pass=false；fine-surface55 mapping COMPLETE但ATE1.439818m，无navigation通过；无本轮 Arena Quick3 成功证据 |
| D2 验证 | NITS | 指定测试绿，仅缺边角用例 | 最新 unittest 命令 exit0，25 tests PASS；scaffold exit0；缺零/倒退 obstacle stamp 的适配层断言测试 |
| D3 卫生 | PASS | 路径已三分，无发现密钥，依赖已 pin | git status 全部 owned 内容未跟踪；四个 runtime revision 为 commit hash；`.gitignore` 排除 external/build/install/integration results等 |
| D4 质量 | BLOCK | 新函数 CCN≥15 | C3 `ros_interface_smoke.main` lizard CC20；C2评分漏洞评审中已修复并复验。jscpd未安装，不安装新工具 |
| D5 设计 | PASS | 同层同目录模式，使用已有扩展点 | Nav2 BasicNavigator、ROS Node、共享 geometry/metrics；两个导航 shell 调用同一 runner，bridge/scan按同一安装约定 |
| D6 平台 | PASS | 未扩建新框架；复用现有扩展点 | 新 CLI main/ROS spin、BoundedNavigator lifecycle override、TF probe进程、Gazebo现有插件均服务计划中仿真/导航集成；无新 ABC/Protocol/registry/hook 框架 |

测试命令（本评审实际执行）:

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -v
bash scripts/build.sh scaffold
lizard -T cyclomatic_complexity=15 -T nloc=80 -T parameter_count=8 integration/scripts src/jt16_bringup/src integration/tests scripts
radon cc -s -n C --no-assert integration/scripts integration/tests scripts
radon mi -s -n B integration/scripts integration/tests scripts
```

MI 未见 <10；radon --no-assert 的 smoke main 为13，而 lizard含assert分支为20。`audit_v0.check` 的 lizard24是单行def引起的解析归属错误：后续模块语句被纳入该函数，不能作为真实复杂度 blocker。不要为该误报改写 full-release audit。

Git diff / cached diff / name-only 均空；真实评审面是未跟踪源码，不是“没有变化”。

## Git 三分

| 路径 | 处置 | 理由 |
|---|---|---|
| `.gitignore`, `LICENSE`, `README.md`, `plan.md` | commit | 忽略规则、许可、需求及使用说明 |
| `dependencies.reference.repos`, `dependencies.runtime.repos`, `dependencies.templates.repos` | commit | 固定上游依赖身份 |
| `docs/**` | commit | 上游锁定和历史/本次评审记录，保留历史失败结论 |
| `scripts/**` | commit | 获取依赖和构建/check入口 |
| `src/**` | commit | ROS包、参数、TF探针 |
| `integration/scripts/**`, `integration/tests/**`, `integration/config/**`, `integration/benchmarks/manifests/**` | commit | 仿真实现、回归、固定场景参数；修完 blocker 后提交 |
| `external/**`, `build/**`, `install/**`, `log/**`, `results/**` | gitignore | 已被忽略的下载及生成文件，不纳入源码 |
| `integration/archive/**`, `integration/build/**`, `integration/install/**`, `integration/log/**`, `integration/deps/**`, `integration/results/**`, `integration/models/**`, `integration/worlds/**`, `integration/benchmarks/generated/**`, `**/__pycache__/**`, `**/*.pyc` | gitignore | 已忽略；保留失败和候选数据，不删除、不覆盖 |

## 代码修改建议

| ID | 必须/建议 | 文件:行 | 函数或符号 | 改法 |
|---|---|---|---|---|
| C3 | 必须 | `integration/tests/ros_interface_smoke.py:19` | `main` | 将已有输出断言移到小检查函数，生命周期保持原样 |
| C4 | 建议 | `integration/scripts/sim_state_adapter.py:49` | `publish_obstacle` | 拒绝零/倒退 observation stamp，并增加对应单测 |

### C2 补丁示意

以下是评审中提出、现已由主agent修复的历史建议，不再是待修改项；最新 completion_metrics 接收terminal，要求SUCCEEDED和有效定位；score_arena也严格检查终态，25 tests已复验通过。本节仅保留审查来龙去脉，不要求新增physical_goal_reached字段。

目标: `navigation_metrics.py`；替换成功判定，两个 runner和score_arena自然复用。保留独立物理到达字段，不能用成功终态掩盖碰撞。

```diff
-def completion_metrics(result,args,elapsed,input_failure):
-    result['scored_success']=bool(result['scoring_valid'] and result['collision'] is False and not input_failure and result['final_goal_distance_m']<=args.goal_tolerance)
+def completion_metrics(result,args,terminal):
+    result['physical_goal_reached']=bool(result['scoring_valid'] and
+        result['collision'] is False and result['final_goal_distance_m']<=args.goal_tolerance)
+    result['scored_success']=bool(result['physical_goal_reached'] and
+        terminal['result']=='SUCCEEDED' and not terminal['input_failure'] and
+        result.get('localization_success') is True)
+    elapsed=terminal['elapsed_s']
 ...
-    completion_metrics(result,args,terminal['elapsed_s'],terminal['input_failure'])
+    completion_metrics(result,args,terminal)
```

补测试: 同一近目标、零碰撞轨迹分别提供 FAILED/TIMEOUT/CANCELED/SUCCEEDED；前3必须scored_success=false。SUCCEEDED但localization_success=false也失败。现有 test_offline_scoring_matches_terminal_failure 的terminal fixture要同步加result字段。Arena汇总不得将 physical_goal_reached 作为成功字段。

### C3 补丁示意

目标: `ros_interface_smoke.py`；抽取已有断言，不增架构或测试门槛。

```diff
+def check_outputs(poses, scans, transforms):
+    assert poses and scans and transforms, 'missing installed-node outputs'
+    assert abs(poses[-1].pose.pose.position.x)<1e-6
+    assert abs(poses[-1].pose.pose.position.z)<1e-6
+    assert any(t.header.frame_id=='odom' and t.child_frame_id=='base_link' for t in transforms)
+    assert scans[-1].header.frame_id=='base_link'
+    assert any(.9<distance<1.1 for distance in scans[-1].ranges)
 ... main output check block ...
-        # above six assertions inline
+        check_outputs(poses, scans, transforms)
```

复跑lizard门槛及安装后的ROS interface smoke；子进程finally清理保留。

### C4 补丁示意

目标: `sim_state_adapter.py`，保留GetEntityState实际观察，不能拿请求位置或请求时间补造证据。

```diff
 ... __init__ ...
+  self.obstacle_stamps={}
 ... publish_obstacle after RESULT_OK ...
+  stamp=response.state.header.stamp.sec+response.state.header.stamp.nanosec*1e-9
+  if stamp<=self.obstacle_stamps.get(index,0.):return
+  self.obstacle_stamps[index]=stamp
```

## Review comments

| ID | 维 | 等级 | 挡住提交 | 位置 | 标题 |
|---|---|---|---|---|---|
| C1 | D1 | BLOCK | 1 | `integration/results/v0/rigid-planar-candidate.I21Ms4/quick-summary.json` | 固定小集尚未通过 |
| C2 | D4 | NITS | 0 | `integration/scripts/navigation_metrics.py:127` | 评分漏洞已修复，待新运行证据 |
| C3 | D4 | BLOCK | 1 | `integration/tests/ros_interface_smoke.py:19` | 新main超过复杂度门槛 |
| C4 | D2 | NITS | 0 | `integration/scripts/sim_state_adapter.py:49` | 观察时间戳异常缺适配层测试 |

### C1. 固定小集尚未通过

- 维: D1；等级: BLOCK；挡住提交: 1。
- 位置: Quick summary及`fine-surface-candidate.YPysAF/world_055`。
- 问题: 已有22成功，但55激活失败、77动作FAILED，不能宣称“小集成功率很高”或完成交付。
- 证据: 22 final distance=.271689m；55 bt_navigator did not become active；77 final distance=9.909427m；fine-surface55最终mapping COMPLETE，但ATE1.439818m，未改善，navigation无通过；本轮无Arena三例验收证据。
- 代码修改: 无，本条为补证据。
- 建议: 修完代码后使用全新固定候选根，冻结代码/二进制，完整运行22/55/77 seed0和Arena quick3的三例，150s不变；独立重放NPZ的碰撞/定位/到达分数，检查真实world/map和TF identity；不得覆盖失败或使用混合版本拼成PASS。
- 处置: 补证据。

### C2. 失败任务可被Arena计为成功

- 维: D4；等级: NITS；挡住提交: 0。
- 位置: `navigation_metrics.py:127`、`score_arena.py:10`。
- 问题: 初读发现失败动作近目标被Arena计成功；评审中已由主agent修复，已关闭代码blocker。
- 证据: 最新completion_metrics及score_arena同时要求SUCCEEDED、scoring_valid、localization_success和collision=false；新增失败终态/Arena聚合测试；本评审复跑25 tests PASS。
- 代码修改: 无，已修复。
- 建议: 冻结修复后评分版本，重跑小集并保留版本身份；旧失败数据不能自动升级为新算法证据。
- 处置: 补证据。

### C3. 新main超过复杂度门槛

- 维: D4；等级: BLOCK；挡住提交: 1。
- 位置: `integration/tests/ros_interface_smoke.py:19`。
- 问题: 新代码范围包含整个smoke main，lizard CC20达到≥15门槛。
- 证据: 本评审lizard实测main nloc53 CC20；radon去掉assert后13不撤销lizard阈值命中。
- 代码修改: 见C3补丁。
- 建议: 仅抽已有输出断言，保留启动/派生速度验证/超时/清理；重新运行质量命令和ROS smoke。
- 处置: 改代码。

### C4. 观察时间戳异常缺适配层测试

- 维: D2；等级: NITS；挡住提交: 0。
- 位置: `integration/scripts/sim_state_adapter.py:49`。
- 问题: 下游已拒绝大的时差，但publish_obstacle允许零/倒退header，真实仿真接口兼容性问题只能到评分时发现。
- 证据: publish_obstacle只检查RESULT_OK；已有测试检查Set失败不发requested pose，无header异常case。
- 代码修改: 见C4补丁。
- 建议: 在适配层拒绝异常stamp；live Arena必须验证GetEntityState header确为仿真时间，不造时间戳。
- 处置: 改代码。

未检查完整BARN/Arena、真机标定、设备通道校准；不提供full V0验收结论。当前模型为理想刚性运动仿真，成功也不能证明真实轮/臂动力学和JT16硬件性能。
