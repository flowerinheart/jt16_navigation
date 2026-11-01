# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **0** |
| **整体等级** | BLOCK |
| 一句话 | 小集六例达标，但遗留导航 smoke 入口必失败 |
| D1 交付 | PASS |
| D2 验证 | PASS |
| D3 卫生 | PASS |
| D4 质量 | BLOCK |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | C1 |
| 代码修改 | 必须 1 处；建议 0 处（见「代码修改建议」） |
| 评论数 | BLOCK=1；NITS=0 |
| 下一步 | 先处理 C1；保留本轮真实通过的证据，不改严格评分 |
| Goal complete | 0 |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`

范围: uncommitted；所有未跟踪 owned source 按新代码审查，不把空 Git diff 当作无改动。

终态: 冻结 Arena Quick-3 与 fresh-mapping BARN Quick-3 使用同一当前源码/runtime，各严格 3/3、零碰撞与过程/输入失败，并具备可信传感、评分、provenance 和 TF 产物。

未执行: 未 commit、未 push、未改代码；仅新增本评审文档。未重跑仿真、完整 benchmark、硬件验收或网络依赖导入。

## 六维打分

| 维 | 等级 | 命中的门槛 | 证据（路径/命令/exit/符号） |
|---|---|---|---|
| D1 交付 | PASS | 终态发生、无范围外、必交达标 | 下列 6 个导航、3 个 fresh mapping 的逐文件检查、NPZ 独立重放、hash/归档/TF 检查均通过；两个 aggregate 与实际成员一致 |
| D2 验证 | PASS | 绿、覆盖新行为、golden 未放宽 | 指定 Python UT 51/51；scaffold build；native 4/4；installed ROS smoke，均 exit 0；时序评分、时间偏移、足迹朝向、无效输入、运动丢失记忆和严格评分有回归覆盖 |
| D3 卫生 | PASS | 每个路径已三分且无密钥、新依赖已 pin、无残留垃圾 | `git status --short`、两个 diff、两个 name-only 与仓库根已检查；依赖检查 14/14；生成物/原始评估结果由 ignore 排除；密钥模式搜索无命中 |
| D4 质量 | BLOCK | 正确性错误：掉调用 | `run_nav2_smoke.sh:29` 未给严格 scorer 所需碰撞几何，导致即使成功到达也必定失败，见 C1；lizard 全 owned Python/C++（含 tests）无门槛警告 |
| D5 设计 | PASS | 同层同目录同模式 | temporal critic 使用已安装 Nav2 `CriticFunction`/pluginlib；参照 sibling `PredictionLayer`、现有扫描投影与 odometry bridge；未重写 MPPI 控制器或 vendor frontend |
| D6 平台 | PASS | 有扩展符号且终态明确允许 | 已授权的 `mppi::critics::TimedObstacleCritic`、原有 `jt16_bringup::PredictionLayer`，通过现有插件扩展点注册；ROS node main、bounded navigator、runner cleanup 为本仿真工作流生命周期，并非新框架 |

### 交付证据逐例核验

证据根 `R = integration/results/v0/timed-mppi-memory-repeat`。下表目录各自的 `nav2-goal.json`、`navigation-sensors.npz`、`navigation-trajectories.npz`、`run-provenance.json`、`tf-audit.json` 均非空并已读取。不是仅依赖目录数量或摘要 PASS。

| R 下的精确导航路径 | elapsed_s | 最小障碍净距 m | 严格结果 |
|---|---:|---:|---|
| `arena/map2_obs05/seed_00` | 131.092 | 0.249985 | PASS |
| `arena/small_warehouse_obs05/seed_01` | 105.154 | 0.317865 | PASS |
| `arena/map5_obs05/seed_01` | 78.876 | 1.564284 | PASS |
| `barn/world_022/navigation/seed_00` | 88.414 | 0.169808 | PASS |
| `barn/world_055/navigation/seed_00` | 65.334 | 0.139697 | PASS |
| `barn/world_077/navigation/seed_00` | 47.924 | 0.502912 | PASS |

六例均满足：`result=SUCCEEDED`，`scored_success/scoring_valid/localization_success=true`，`collision=false`，`input_failure=null`，有限且 0≤elapsed≤150，`navigation_timeout_s=150`。实际配置启用 temporal critic，禁用静态 swept-future layer。传感 NPZ 使用 `allow_pickle=False` 打开全部数组，moving input 消息/点数量与 JSON 相符，运动点有限。

独立从六份 trajectory NPZ、归档参数和实际几何重新调用 `score_navigation`，所有返回指标与 JSON 相符（数值容差 1e-8）。逐个 source 的当前文件与 `executed-inputs` hash、runtime 二进制 hash、实际输入及归档 hash 全部相符。九份 mapping/navigation 的 source/runtime 字典完全一致。六份 TF audit 无重复发布者，`map->odom` 仅 AMCL，`odom->base_link` 仅 FAST-LIO bridge。

下列 mapping 路径各自的 `mapping-result.json`、`map.pcd`、`nav-map/map.yaml`、`nav-map/map.pgm`、`map-metrics.json`、`map-bundle-metadata.json`、`run-provenance.json`、`mapping-trajectories.npz`、`input-diagnostics.npz` 均非空并已读取；map bundle 五项 hash 与实物相符。

| R 下的 mapping 路径 | 完成路径点 | 独立重放 ATE RMSE m | occupied F1 |
|---|---:|---:|---:|
| `barn/world_022/mapping` | 14/14 | 0.059103 | 0.884219 |
| `barn/world_055/mapping` | 19/19 | 0.073412 | 0.940545 |
| `barn/world_077/mapping` | 10/10 | 0.044855 | 0.909903 |

三例 `terminal_state=COMPLETE`。ATE 从 mapping GT/LIO NPZ 按现有配对/初始航向定义独立重算，与结果一致；四类 input diagnostics 均非空、有限、时间严格递增。F1 已读取并核验对应 map 来源/hash，本轮未另行重算 F1。

`R/arena/arena-summary.json` 精确三例，strict success_rate=1、collision_rate=0、unscorable_runs=0；`R/quick-summary.json` 精确 worlds 22/55/77，run_count=3、promotion_gate_pass=true。BARN 路径采集是 GT 驱动的规定采集轨迹，不是自主探索；导航目标/控制没有 GT 回退。IMU 模拟仍利用理想真值生成惯性观测，不构成硬件独立验证。

### 本轮实际执行的验证

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -q
bash scripts/build.sh scaffold
bash -c 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --base-paths src --packages-select jt16_bringup --event-handlers console_direct+; colcon test-result --test-result-base build/jt16_bringup/test_results --verbose'
bash -c 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; ROS_DOMAIN_ID=96 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout -k 3s 25s /usr/bin/python3 integration/tests/ros_interface_smoke.py'
lizard -w -T cyclomatic_complexity=14 -T nloc=80 -T parameter_count=8 integration/scripts/*.py integration/tests/*.py scripts/*.py src/jt16_bringup/src/*.cpp src/jt16_bringup/test/*.cpp
radon cc -s -n C --no-assert integration/scripts/*.py integration/tests/*.py scripts/*.py
radon mi -s -n B integration/scripts/*.py integration/tests/*.py scripts/*.py
/usr/bin/python3 scripts/check_dependencies.py
```

指定四条验证全部 exit 0。Native 输出 `4 tests, 0 errors, 0 failures, 0 skipped`；installed check 输出 `PASS installed bridge/TF/scan and LiDAR motion forecast interfaces`。lizard 无警告；radon 最大 CC 为 `MotionPredictor.update` 的 14，未达 15。radon 显示 `run_nav2_goal.py` MI=14.16、`test_regressions.py` MI=0.79；测试文件指标较低，未把无可比较旧版的文件级指标当作“本次变差”的证据。jscpd 未安装，not run；未安装新工具，重复逻辑由人工阅读判断。

## Git 三分

按 `git ls-files --others --exclude-standard` 枚举的路径归类；以下目录项覆盖其全部当前未跟踪文件，不包含已忽略的数据/第三方源码。空 staged/unstaged diff 不改变未跟踪源码审查范围。

| 路径 | 处置 | 理由 |
|---|---|---|
| `.gitignore`、`LICENSE`、`README.md`、`plan.md` | commit | 自有项目规则、许可、使用说明和需求 |
| `dependencies.runtime.repos`、`dependencies.reference.repos`、`dependencies.templates.repos` | commit | 外部源均为 exact commit pin |
| `docs/initial-workspace-README.md`、`docs/sources.initial.lock.json` | commit | 保存原始下载和迁移记录，不冒充新结果 |
| `docs/reviews/*.md`（含本报告） | commit | 保留历次 source-specific review/验证历史，不覆盖旧 BLOCK |
| `integration/.gitignore`、`integration/README.md`、`integration/plan.md` | commit | 仿真原型的规则与历史说明 |
| `integration/config/README.md`、`integration/config/jt16_instant.yaml` | commit | 传感/FAST-LIO 仿真配置 |
| `integration/benchmarks/manifests/{arena6,arena_quick3,barn30,quick3}.json` | commit | 固定 public 场景、种子和通过闸门 |
| `integration/scripts/*` | commit | 自有采集、建图、导航、诊断、评分、provenance；C1 文件须先处理 |
| `integration/tests/{test_regressions,test_motion_prediction,ros_interface_smoke}.py` | commit | 纯逻辑与 installed ROS 接口回归 |
| `scripts/{build,fetch_dependencies}.sh`、`scripts/check_dependencies.py` | commit | 复用 colcon/vcstool，不引入新构建框架 |
| `src/jt16_bringup/{CMakeLists.txt,package.xml,prediction_plugins.xml,timed_critic_plugins.xml}` | commit | 自有包、原生插件注册与导出 |
| `src/jt16_bringup/config/*`、`src/jt16_bringup/include/jt16_bringup/*`、`src/jt16_bringup/src/*`、`src/jt16_bringup/test/*` | commit | 配置、实现、公开头及 native tests |

当前没有待 revert/gitignore 的可见脏路径。`external/`、`build/`、`install/`、`log/`、`integration/results/`、生成模型/世界、缓存由已提交候选 ignore 规则排除；不得把 benchmark NPZ/log/第三方下载追加到源码提交。

## 代码修改建议

| ID | 必须/建议 | 文件:行 | 函数或符号 | 改法（一句） |
|---|---|---|---|---|
| C1 | 必须 | `integration/scripts/run_nav2_smoke.sh:29`；`integration/tests/test_regressions.py:141` | smoke shell main；`test_runners_reject_existing_evidence_before_launch`、`test_navigation_defaults_generate_mppi_before_children_launch` | 删除无业务/文档调用且必失败的旧 smoke 入口，并删除两个测试列表引用；保留已验证的 ROS 接口 smoke 和 frozen Public 导航入口 |

### C1 补丁示意

- 目标: `integration/scripts/run_nav2_smoke.sh` 与上述两个 regression 函数。
- 动作: 删除废弃入口和对应测试列表条目；不要放宽严格 scorer 或新增第三套非 Public 导航接受规则。

```diff
diff --git a/integration/scripts/run_nav2_smoke.sh b/integration/scripts/run_nav2_smoke.sh
deleted file mode 100644
--- a/integration/scripts/run_nav2_smoke.sh
+++ /dev/null
@@
-# 删除该完整旧入口（29 行）；没有文档或生产调用。
diff --git a/integration/tests/test_regressions.py b/integration/tests/test_regressions.py
--- a/integration/tests/test_regressions.py
+++ b/integration/tests/test_regressions.py
@@
-                         ('run_nav2_smoke.sh',[str(output),str(evidence)]),
@@
-        for script in ['run_arena_navigation.sh','run_barn_navigation.sh','run_nav2_smoke.sh']:
+        for script in ['run_arena_navigation.sh','run_barn_navigation.sh']:
```

这是方向明确的补丁示意，不是可直接 apply 的完整删除 patch。删除源码也会改变 source identity：不得改旧 provenance 来假装相同源码；后续评审须明确区分“六例原冻结版本已经通过”与修改后的当前版本，以及按实际影响重新验证/冻结的证据。

## Review comments

| ID | 维 | 等级 | 挡住提交 | 位置 | 标题 |
|---|---|---|---|---|---|
| C1 | D4 | BLOCK | 1 | `integration/scripts/run_nav2_smoke.sh:29` | 导航 smoke 缺评分几何，成功也必失败 |

### C1. 导航 smoke 缺评分几何，成功也必失败

- 维: D4 质量
- 等级: BLOCK
- 挡住提交: 1
- 位置: `integration/scripts/run_nav2_smoke.sh:29`；`integration/scripts/navigation_metrics.py:158`
- 问题: 旧 shell 入口仍调用严格 `run_nav2_goal.py`，但仅传 `--simulation-world`；此选项只用于 provenance，没有提供评分分支需要的 Arena scenario/world 或 BARN world。即使 Nav2 成功且定位正常，也因 missing collision geometry 被标为失败并 exit 1。
- 证据: 已阅读 caller 与 `score_navigation`。独立无 ROS 重现使用两点真值轨迹、终点精确 0.3m、`result=SUCCEEDED`、elapsed=1、有效新鲜 AMCL、该入口对应的 `arena_scenario=None/barn_world=None`，得到 `scoring_valid=False`、`scoring_error='missing collision geometry'`、`collision=None`、`scored_success=False`。`run_nav2_goal.py` 最后按严格 scored_success exit。`rg` 显示此 shell 仅被两个静态 regression 列表引用，未见业务/文档调用。
- 代码修改: 见「代码修改建议」C1。
- 建议: 删除废弃 shell 及两处列表引用；重新跑 51-test Python 集、build/native/installed check，并按 source identity 变化处理后续冻结评审。不要调整目标容差、碰撞规则或把未评分成功算成成功。
- 处置: 改代码

未检查完整 benchmark、真实 JT16 校准与噪声、轮式动力学或网络导入；没有启动昂贵仿真。
风险：六例只证明规定小仿真闸门；常速度/有限遮挡记忆、rotation shim 阶段与硬件传感几何不构成广泛动态安全保证。
