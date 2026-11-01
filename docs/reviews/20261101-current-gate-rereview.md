# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **1** |
| **整体等级** | CLEAN |
| 一句话 | 当前同版六例及三次新建图齐全，小仿真门槛通过 |
| D1 交付 | PASS |
| D2 验证 | PASS |
| D3 卫生 | PASS |
| D4 质量 | PASS |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | 无 |
| 代码修改 | 必须 0 处；建议 0 处（见「代码修改建议」） |
| 评论数 | BLOCK=0；NITS=0 |
| 下一步 | 按 Git 三分后可用的 title `feat: validate LiDAR-only temporal MPPI small simulation gate`；提交和全量评测仍需用户授权 |
| Goal complete | 1（仅本调用单的小仿真交付终态） |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`

范围: uncommitted；全部自有未跟踪源码作为新代码，包含连接的调用方、测试、配置和插件导出，不评审 vendor 内部代码。由新的独立 agent 执行，不复用此前 reviewer 的结论。

终态: 官方 Hesai FAST-LIO 与获准的仅 LiDAR temporal MPPI 方案，在冻结 Arena Quick3 和三次新建图的 BARN Quick3 上以当前同一 source/runtime 严格通过，可用于小仿真交付。

需求: `README.md`、`plan.md`。必交文件根目录按调用方更新为 `integration/results/v0/timed-mppi-startup-pilot/`；目录名 pilot 不替代文件、评分和身份检查。

范围外: 感知识别模型、GT 导航、场景/种子/速度/评分/NAV150 修改、完整评测、硬件验收、commit/push。BARN 固定路径建图采集使用 GT 驱动，仅是规定的数据获取流程，不是自主探索能力证明；导航不使用 GT 位姿控制。理想化 IMU 生成也不是硬件传感器验证。

未执行: 未 commit、未 push、未改实现代码；只新增本报告。仿真运行期间仅只读检查，结束后独立运行测试，不构建或变更运行库。

## 六维打分

| 维 | 等级 | 命中的门槛 | 证据（路径/命令/exit/符号） |
|---|---|---|---|
| D1 交付 | PASS | 终态发生、必交达标、无范围外功能 | 当前 root 的 `arena/arena-summary.json` 严格 3/3、碰撞率 0、不可评分数 0；`quick-summary.json` 三个冻结 world、NAV150、promotion_gate_pass=true。六例 NPZ 独立重评分逐项等于 JSON，三次新建图 COMPLETE；九次 source/runtime 相同且匹配当前和归档 |
| D2 验证 | PASS | 绿、覆盖新行为、golden 未放宽 | 独立复跑 Python 54 tests OK、exit 0；native 5 tests、0 errors/0 failures/0 skipped、exit 0。读取指定 Release 构建成功日志。覆盖映射 TIMEOUT 非零退出、空/缺 PCD 拒绝、不完整 bundle 拒绝续跑并保留历史输出、运动记忆过期及风险数值等价 |
| D3 卫生 | PASS | 每个路径已三分，无发现密钥，新依赖已 pin | `git status --short` 和逐文件 untracked 列表；tracked staged/unstaged diff 均为空。14 个依赖 checkout 独立验证 PASS，Hesai FAST-LIO/driver 工作区 clean；生成物已 ignore；密钥模式检查未命中 |
| D4 质量 | PASS | 无发现正确性 BLOCK、复杂度/长度跨门槛或足够屎山信号 | Lizard 全部自有 Python/C++ 无 CCN≥15、nloc>80、参数>8 警告；核心时间/坐标/新鲜度/退出逻辑与调用方审阅通过。优化使用最小平方距离后一次 sqrt，原风险评分 native 等价测试通过；没有换评分门槛 |
| D5 设计 | PASS | 同层同目录同模式，复用已有模块/基类/约定 | `TimedObstacleCritic` 使用 Nav2 `CriticFunction`/pluginlib；预测复用 `MotionPredictor`；导航保留官方 MPPI、RotationShim、costmap/CollisionMonitor，未另造规划框架。读取 sibling PredictionLayer、TF probe、配置与安装接口 |
| D6 平台 | PASS | 框架扩展已获授权，走现有扩展点 | `mppi::critics::TimedObstacleCritic::{initialize,score}`、`PredictionLayer`、`tf_publisher_probe::main`、ROS 节点和 bounded navigator/runner 生命周期；XML/CMake 导出与已安装运行一致；未添加自有 ABC/Protocol/插件框架 |

指定测试与检查记录：

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -q
# 本 reviewer 独立复跑：54 tests，OK，exit 0。

bash scripts/build.sh scaffold
# 不重复构建；读取 log/latest_build/jt16_bringup/stdout.log、streams.log。
# Release 构建/安装成功，native 插件和可执行文件齐全。

bash -c 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --base-paths src --packages-select jt16_bringup --event-handlers console_direct+; colcon test-result --test-result-base build/jt16_bringup/test_results --verbose'
# 本 reviewer 独立复跑：5 tests，0 errors，0 failures，0 skipped，exit 0。
# 同时读取指定 /tmp/jt16-final-gate-native-tests.log 的绿记录。

lizard -w -T cyclomatic_complexity=14 -T nloc=80 -T parameter_count=8 integration/scripts/*.py integration/tests/*.py scripts/*.py src/jt16_bringup/src/*.cpp src/jt16_bringup/test/*.cpp
radon cc -s -n C --no-assert integration/scripts/*.py integration/tests/*.py scripts/*.py
radon mi -s -n B integration/scripts/*.py integration/tests/*.py scripts/*.py
/usr/bin/python3 scripts/check_dependencies.py
```

Radon：`MotionPredictor.update` CC 14；`run_nav2_goal.py` MI 14.16；测试集合 `test_regressions.py` MI 0.00。没有可比较的 tracked 旧版本，不虚构 MI 降幅；测试集合的低 MI 本身不代替函数门槛与逻辑审阅。jscpd 未安装，not run，未安装额外工具；人工检查未发现阻塞的新逻辑克隆或过度抽象。

### 当前源码的独立证据核对

以下全部来自当前 root 原始产物，没有复制旧成功结果：

| 冻结导航样例 | 动作 | 时间 s | 最小间隙 m | 最终 GT 目标距离 m |
|---|---|---:|---:|---:|
| Arena warehouse / obs5 / seed1 | SUCCEEDED | 113.612 | 0.187625 | 0.016352 |
| Arena map2 / obs5 / seed0 | SUCCEEDED | 143.438 | 0.163012 | 0.091272 |
| Arena map5 / obs5 / seed1 | SUCCEEDED | 79.258 | 1.525463 | 0.058723 |
| BARN22 / seed0 | SUCCEEDED | 149.304 | 0.182592 | 0.241547 |
| BARN55 / seed0 | SUCCEEDED | 64.976 | 0.122301 | 0.124103 |
| BARN77 / seed0 | SUCCEEDED | 41.800 | 0.570634 | 0.091961 |

六例均 strict scored_success=true、collision=false、input_failure=null、scoring_valid=true、localization_success=true，NAV150 不变。独立读取 `run-provenance.json` 的真实参数，调用 `score_navigation` 重放六个 `navigation-trajectories.npz`，每项结果与 JSON 一致。六例 temporal critic 启用、静态预测层关闭，真实 scan/costmap 检查仍保留。

| 当前新建图 | 路径点 | ATE RMSE m | occupied F1 | occupied IoU |
|---|---:|---:|---:|---:|
| BARN22 | 14/14 | 0.054993 | 0.886539 | 0.796202 |
| BARN55 | 19/19 | 0.116363 | 0.946719 | 0.898829 |
| BARN77 | 10/10 | 0.038029 | 0.895281 | 0.810415 |

三个 `barn/world_0XX/mapping/{mapping-result.json,map-bundle-metadata.json,map-metrics.json}` 非空且 COMPLETE。ATE 从各自 `mapping-trajectories.npz` 独立按时间配对和初始坐标对齐重算，与原值一致。五个 bundle 成员（PCD、map YAML、map PGM、FAST-LIO config、mount）内容哈希通过；`input-diagnostics.npz` 的 raw/adapted IMU、LIO 姿态、reported twist 以及 gt/lio 轨迹均非空。

全部九次运行的 **76 source hashes / 33 runtime hashes** 完全相同；逐项验证当前源、当前运行库、source 执行归档、run-input 当前文件和执行归档哈希。测试复跑后再次验证未变更。六个 TF audit 独占 `map->odom` 的 AMCL 和 `odom->base_link` 的 LIO bridge，均 PASS。

这修复了此前 D1「当前源码六例缺失」的阻塞项；历史 BLOCK 报告原样保留，不将旧版本输出换标签。

### 不扩大结论的风险记录

此前同源码 `timed-mppi-current-gate/arena/map2_obs05/seed_00` 出现导航前 PROCESS_FAILURE。独立日志检查确认 MPPI/critics 配置完成后约 0.1 s，`/controller_server/change_state` 应答发生 DDS timeout，导航生命周期未完成；AMCL 和 TF 正常。新 root 默认源码/参数下未复现，未改 RMW/评分或添加自动重试，但这不是已修复启动可靠性的证明。失败产物保留，不能声称所有尝试成功。

BARN22 只有 **0.696 s** 的 NAV150 余量；一次冻结小集通过不能推导稳定成功率，也不能声称耗时、ATE 或全部指标均改善。当前没有 previous released V0，不作完整版本非回归声明。常速预测和最多 0.5 s 记忆不保证转向或长期遮挡安全；RotationShim/恢复行为不执行 temporal critic。以上限制不改变已实际通过的本次小仿真调用单，但必须随交付保留。

## Git 三分

以下互不重叠路径组覆盖当前全部自有脏文件；commit 仅为分类，不是本轮提交授权。

| 路径 | 处置 | 理由 |
|---|---|---|
| `.gitignore`、`integration/.gitignore` | commit | 排除依赖、缓存、生成物与历史评测数据 |
| `LICENSE` | commit | 自有代码许可证，不覆盖 vendor 许可证 |
| `README.md`、`plan.md`、`integration/README.md`、`integration/plan.md` | commit | 需求、当前状态及历史原型边界 |
| `dependencies.runtime.repos`、`dependencies.reference.repos`、`dependencies.templates.repos` | commit | 精确 commit 固定依赖 |
| `docs/initial-workspace-README.md`、`docs/sources.initial.lock.json` | commit | 初始状态与来源记录 |
| `docs/reviews/*.md` | commit | 保留历史 review/修复记录及本报告 |
| `integration/benchmarks/manifests/*.json` | commit | 冻结场景、种子及门槛 |
| `integration/config/README.md`、`integration/config/jt16_instant.yaml` | commit | FAST-LIO 仿真配置及限制 |
| `integration/scripts/*.py`、`integration/scripts/*.sh` | commit | 自有转换、采集、运行与评分代码 |
| `integration/tests/*.py` | commit | 固定数据和已安装接口回归 |
| `scripts/build.sh`、`scripts/check_dependencies.py`、`scripts/fetch_dependencies.sh` | commit | 构建与依赖管理 |
| `src/jt16_bringup/CMakeLists.txt`、`src/jt16_bringup/package.xml` | commit | 编译、依赖与安装接口 |
| `src/jt16_bringup/config/*`、`src/jt16_bringup/include/jt16_bringup/*.hpp`、`src/jt16_bringup/src/*.cpp`、`src/jt16_bringup/test/*.cpp`、`src/jt16_bringup/*plugins.xml` | commit | 自有配置、native 实现、测试及授权插件导出 |

external、integration/results/models/worlds/benchmarks/generated、build/install/log/deps/archive 和 Python 缓存已 ignore，不是未处理的脏文件。无需 revert/gitignore 的剩余路径；不 add/restore/commit。

## 代码修改建议

无代码修改（本轮是补证据后的重新评审，无评论）。

## Review comments

无

未跑完整 benchmark、硬件验收或真实 JT16 校准；未再次构建、未运行缺失的 jscpd，也未提交/推送。
风险：历史 DDS 启动失败、BARN22 极小超时余量和常速/遮挡/恢复限制仍在，CLEAN 只表示本调用单的小仿真门槛通过。
