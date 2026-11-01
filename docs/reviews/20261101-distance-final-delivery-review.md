# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **0** |
| **整体等级** | BLOCK |
| 一句话 | 映射入口已修复，但当前源码的六例交付证据未齐 |
| D1 交付 | BLOCK |
| D2 验证 | PASS |
| D3 卫生 | PASS |
| D4 质量 | PASS |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | C1 |
| 代码修改 | 必须 0 处；建议 0 处（见「代码修改建议」） |
| 评论数 | BLOCK=1；NITS=0 |
| 下一步 | 先处理 C1：冻结当前源码，在新根目录补齐六例与三次新建图 |
| Goal complete | 0 |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`

范围: uncommitted；全部未跟踪的自有源码作为新代码，包含调用方、配置、测试和插件导出；不评审 vendor 仓库内部代码。

终态: 官方 Hesai FAST-LIO 与仅 LiDAR 的获准 temporal MPPI 方案，在冻结 Arena Quick3 与新建图 BARN Quick3 上严格通过，代码和证据可用于小仿真交付。

需求: `plan.md`、`README.md`。

范围外: 感知识别模型、GT 导航、场景/种子/速度/评分/NAV150 修改、完整评测、硬件验收、commit/push。

未执行: 未 commit、未 push、未改实现代码。仅新增本报告；仿真运行期间不写源代码或运行构建。

## 六维打分

| 维 | 等级 | 命中的门槛 | 证据（路径/命令/exit/符号） |
|---|---|---|---|
| D1 交付 | BLOCK | 必交产物 missing / 未跑，当前终态未发生 | `integration/results/v0/timed-mppi-distance-repeat/quick-summary.json` 不存在；BARN55/77 未运行；现有四个导航结果属于映射入口修复前的源码身份，当前 provenance 校验正确拒绝它们 |
| D2 验证 | PASS | 绿且覆盖新行为，未放宽 golden | 独立复跑 Python 54 tests，exit 0；独立复跑 native 5 tests，0 errors/0 failures；读取 scaffold 构建日志成功记录。新回归覆盖映射 TIMEOUT 非零退出、缺失/空 PCD 拒绝、已有不完整映射拒绝且保留历史数据 |
| D3 卫生 | PASS | 脏路径已三分，无发现的密钥，依赖已 pin | `git status --short`、空 staged/unstaged tracked diff；全部自有未跟踪文件分类如下；14 个依赖 checkout hash 检查通过；Hesai FAST-LIO/driver 工作区无变更；生成物已 ignore |
| D4 质量 | PASS | 当前无正确性 BLOCK、函数复杂度/长度跨门槛或足够的屎山信号 | 映射退出与 resume 错误在安全边界修复后已独立确认。Lizard 全部自有 Python/C++ 函数无 CCN≥15、nloc>80、参数>8 警告；时间匹配/方向/过期/空输入/原评分数值等价测试通过 |
| D5 设计 | PASS | 使用现有模块、基类和调用约定 | 新 critic 使用 Nav2 `CriticFunction` 与 pluginlib；预测复用 `MotionPredictor`；导航调用现有 Nav2/RotationShim，未另造规划框架；读取 sibling `PredictionLayer`、TF probe 与配置/CMake 安装接口 |
| D6 平台 | PASS | 插件扩展已由用户授权，走现有扩展点 | `mppi::critics::TimedObstacleCritic::{initialize,score}`、`PredictionLayer`、`tf_publisher_probe::main`、ROS 节点和 bounded navigator/runner 生命周期；插件 XML/CMake 导出与运行测试一致；未引入新 ABC/Protocol 或自有插件框架 |

指定验证执行/读取记录：

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -q
# 独立复跑：54 tests，OK，exit 0。

bash scripts/build.sh scaffold
# 本轮不再次构建；读取 log/latest_build/jt16_bringup/
# stdout.log、streams.log 的成功构建/安装记录，返回 0。

bash -c 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --base-paths src --packages-select jt16_bringup --event-handlers console_direct+; colcon test-result --test-result-base build/jt16_bringup/test_results --verbose'
# 独立复跑：5 tests，0 errors，0 failures，0 skipped，exit 0。

lizard -w -T cyclomatic_complexity=14 -T nloc=80 -T parameter_count=8 integration/scripts/*.py integration/tests/*.py scripts/*.py src/jt16_bringup/src/*.cpp src/jt16_bringup/test/*.cpp
radon cc -s -n C --no-assert integration/scripts/*.py integration/tests/*.py scripts/*.py
radon mi -s -n B integration/scripts/*.py integration/tests/*.py scripts/*.py
```

Radon：`MotionPredictor.update` CC 14；`run_nav2_goal.py` MI 14.16；`test_regressions.py` MI 0.79。后者为测试集合，本轮不虚构不存在的历史 MI 降幅。jscpd 未安装，not run；人工检查未发现需要阻止本次小仿真交付的新重复逻辑。未安装额外检查工具。

### 已有证据的独立核对

以下是 **入口修复前的 interim distance-repeat 候选**，不是当前源码六例验收：

| 样例 | 严格动作结果 | 时间 s | 最小障碍间隙 m | 最终目标距离 m |
|---|---|---:|---:|---:|
| Arena warehouse / obs5 / seed1 | SUCCEEDED | 105.272 | 0.187379 | 0.159510 |
| Arena map2 / obs5 / seed0 | SUCCEEDED | 135.588 | 0.216330 | 0.053935 |
| Arena map5 / obs5 / seed1 | SUCCEEDED | 78.968 | 1.525355 | 0.075849 |
| BARN22 / seed0 | SUCCEEDED | 148.460 | 0.197068 | 0.255059 |

独立从上述四个 `navigation-trajectories.npz` 调用 `score_navigation` 回放，严格成功、碰撞、定位、间隙、目标距离与 JSON 一致，均无 input_failure。Arena 汇总真实为 3/3、零碰撞、零不可评分结果。Arena TF audit 显示 AMCL 独占 `map->odom`、LIO bridge 独占 `odom->base_link`。

入口修复前，三例 Arena 的当前源/运行库/源归档/输入归档哈希全部核验通过：76 sources、33 runtime hashes。映射入口修改后，同一 root 的当前源码校验报 `candidate changed: integration/scripts/run_barn30_mapping.sh; use a new output root`；这是有效的身份隔离，不能忽略它或重写旧归档。

BARN22 新建图 `COMPLETE`，14/14 waypoint，ATE RMSE 0.061012 m，occupied F1 0.884814、IoU 0.793423；独立确认五个 map-bundle 文件哈希，轨迹包含 gt/lio，诊断 NPZ 包含 raw/adapted IMU、LIO 姿态与报告 twist。BARN55/77 与其新建图产物缺失，不按失败任务伪造数字，也不把 BARN22 当作 BARN3/3。

## Git 三分

以下互不重叠的路径组覆盖全部当前自有未跟踪文件；`commit` 是交付分类，不是授权本轮提交。

| 路径 | 处置 | 理由 |
|---|---|---|
| `.gitignore`、`integration/.gitignore` | commit | 排除依赖、构建、历史数据和生成物 |
| `LICENSE` | commit | 自有代码许可证，不覆盖 vendor 许可证 |
| `README.md`、`plan.md`、`integration/README.md`、`integration/plan.md` | commit | 需求、现状与历史原型边界 |
| `dependencies.runtime.repos`、`dependencies.reference.repos`、`dependencies.templates.repos` | commit | 精确 commit 固定依赖 |
| `docs/initial-workspace-README.md`、`docs/sources.initial.lock.json` | commit | 初始状态和来源身份 |
| `docs/reviews/*.md` | commit | 历史 review/修复记录与本报告，保留原 BLOCK |
| `integration/benchmarks/manifests/*.json` | commit | 冻结场景/种子/评分入口 |
| `integration/config/README.md`、`integration/config/jt16_instant.yaml` | commit | 仿真 FAST-LIO 配置与限制 |
| `integration/scripts/*.py`、`integration/scripts/*.sh` | commit | 自有采集、转换、运行和评分实现 |
| `integration/tests/*.py` | commit | 固定数据与已安装 ROS 接口回归 |
| `scripts/build.sh`、`scripts/check_dependencies.py`、`scripts/fetch_dependencies.sh` | commit | 构建与依赖管理 |
| `src/jt16_bringup/CMakeLists.txt`、`src/jt16_bringup/package.xml` | commit | 编译、依赖与安装接口 |
| `src/jt16_bringup/config/*`、`src/jt16_bringup/include/jt16_bringup/*.hpp`、`src/jt16_bringup/src/*.cpp`、`src/jt16_bringup/test/*.cpp`、`src/jt16_bringup/*plugins.xml` | commit | 自有配置、native 实现、测试与现有框架插件导出 |

`external/`、`integration/results/`、`integration/models/`、`integration/benchmarks/generated/`、build/install/log/deps/archive 与 `__pycache__` 已由 ignore 规则排除，不是待 revert/gitignore 的脏路径。未发现需撤销的范围外改动。

## 代码修改建议

无代码修改（本轮剩余工作是补证据）。

本轮提出的映射退出/调用方错误已在安全边界修复：`follow_barn_path.py` 仅 COMPLETE 退出 0；`run_barn30_mapping.sh` 不再跳过已有不完整 bundle，也在新 mapping 失败时停止；`run_barn_mapping.sh` 缺失或空 PCD 退出非零。独立 54-test 复跑确认相应回归通过，因此不把已修复项再次列为必须改代码评论。

## Review comments

| ID | 维 | 等级 | 挡住提交 | 位置 | 标题 |
|---|---|---|---|---|---|
| C1 | D1 | BLOCK | 1 | `integration/results/v0/timed-mppi-distance-repeat/quick-summary.json` | 当前源码的六例证据缺失 |

### C1. 当前源码的六例证据缺失

- 维: D1 交付
- 等级: BLOCK
- 挡住提交: 1
- 位置: `integration/results/v0/timed-mppi-distance-repeat/quick-summary.json`
- 问题: 现有 Arena3 与 BARN22 属于入口修复前身份，BARN55/77 未运行，未达到调用单要求的同一当前源码 Arena3+BARN3 和三次新建图。
- 证据: quick-summary 文件不存在；独立 provenance 校验拒绝已修改的 mapping runner；BARN55/77 mapping-result/map bundle/nav2-goal 缺失；旧四例 NPZ 回放虽通过但不足以满足最终门槛。
- 代码修改: 无
- 建议: 冻结当前源与运行库，在全新 root 顺序采集 Arena warehouse/map2/map5 和 BARN22/55/77（seed0、各自新建图 COMPLETE，再 NAV150），单例不通过即停；不得运行完整集。补齐并检查全部 mapping-result/map-bundle/map-metrics，执行 `python integration/scripts/score_arena.py --results NEW_ROOT/arena` 与 `python integration/scripts/score_quick3.py --root NEW_ROOT/barn --output NEW_ROOT/quick-summary.json`，回放六例 NPZ 并验证每例当前源/运行库/执行归档/输入/TF 身份一致，再发起新的最终交付评审。不要覆盖或修饰本轮旧证据。
- 处置: 补证据

未检查完整 benchmark、硬件和真实 JT16 校准，也未再次构建或运行 jscpd；本轮只接受已有小仿真证据的范围。
风险：constant-velocity + 最多 0.5 s 的运动记忆不保证转向/长期遮挡安全，RotationShim/恢复不执行 temporal critic；当前六例最终门槛尚未完成。
