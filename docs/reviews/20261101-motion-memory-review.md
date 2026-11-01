# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **0** |
| **整体等级** | BLOCK |
| 一句话 | 最终六例证据未齐，BARN失败分支仍返回成功 |
| D1 交付 | BLOCK |
| D2 验证 | PASS |
| D3 卫生 | PASS |
| D4 质量 | BLOCK |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | C1,C2 |
| 代码修改 | 必须 1 处；建议 0 处（见「代码修改建议」） |
| 评论数 | BLOCK=2；NITS=0 |
| 下一步 | 先修 C2，再按 C1 完成同版本小集证据 |
| Goal complete | 0 |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`
范围: uncommitted，所有未跟踪 owned source 按新代码评审；重点为运动确认保留、Gazebo批量移动、观测心跳、严格退出和运行身份。
终态: 冻结 Arena Quick3 与 BARN Quick3 同一最终源码/运行时各严格3/3，建图、传感器和评分可信。
需求: `plan.md` 和调用消息；范围外为全量评测、硬件、commit/push、场景/种子/速度/NAV150/评分变更。
未执行: 未 commit、未 push、未改源码/结果/build/install；只新增本报告。

## 六维打分

| 维 | 等级 | 命中的门槛 | 证据 |
|---|---|---|---|
| D1 交付 | BLOCK | 终态未发生；必交产物未跑齐 | 当前 `timed-mppi-motion-memory-pilot` 仍是单例试验，未提供同一最终身份的 Arena3+BARN3及新建图证据；旧候选结果不能替代 |
| D2 验证 | PASS | 绿且覆盖新行为，golden未放宽 | 独立执行 `PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -q`，50 tests，OK，exit0；断续追踪、不刷新确认时间、第二运动体不丢/重复、移动失败均有断言。原生XML为3+1 tests、0 failures/0 errors；本轮未重跑原生/ROS smoke |
| D3 卫生 | PASS | 每个路径已三分，无密钥、新依赖已pin | `git status --short` 为12个未跟踪顶层项，展开90文件；无tracked diff/staged diff。源码/测试/文档/锁定依赖归commit，数据和生成目录已有ignore |
| D4 质量 | BLOCK | 正确性错误：失败状态退出0 | `run_barn_navigation.sh:14–22` 缺失有效mapping-result时写失败JSON，却 `exit 0`。lizard全owned Python/CPP及测试无警告；radon最高 MotionPredictor.update C(14)，未达CCN15 |
| D5 设计 | PASS | 同层同目录同模式，复用现有扩展点 | MotionPredictor沿用track/history与共享几何helper；预测节点沿用ROS PointCloud2；原生评分插件沿用Nav2 CriticFunction/pluginlib；Gazebo动作和Get实际观测分离 |
| D6 平台 | PASS | Goal明确允许扩框架，使用现有扩展点 | `mppi::critics::TimedObstacleCritic`、plugin XML/CMake export已获授权；`Adapter`、`ObstaclePredictor`、`BoundedNavigator`及shell lifecycle为本任务进程入口，不另造框架 |

工具：lizard运行所有 `integration/scripts/*.py integration/tests/*.py scripts/*.py src/jt16_bringup/src/*.cpp src/jt16_bringup/test/*.cpp`，阈值CC14/nloc80/参数8，无警告；radon CC运行四个重点Python模块。jscpd未安装、未安装工具。读取 `log/build_2026-11-01_13-13-01/jt16_bringup/stdout_stderr.log` 的Release build/install记录，未重建影响live sim。

具体数据流检查：保留motion的时间锚始终是最后一次确认stamp，而非补发stamp；moving_surfaces先外推到当前scan stamp，插件再加pose/scan offset及第一个rollout dt，不重复外推。unobserved_motion通过已匹配track身份避免同一上一帧track重复保留。预测输入只来自scan/FAST-LIO；GT只用于场景动作、离线评分/观测，未接入控制插件。

## Git 三分

以下分组覆盖 `git ls-files --others --exclude-standard` 的全部90文件；目录项指其中全部未跟踪文件，不包含已ignore的运行产物。

| 路径 | 处置 | 理由 |
|---|---|---|
| `.gitignore`, `integration/.gitignore` | commit | 生成数据、日志、build/install及依赖排除规则 |
| `LICENSE`, `README.md`, `plan.md`, `integration/README.md`, `integration/plan.md` | commit | 授权和开发/验证文档，最终交付前须同步真实状态 |
| `dependencies.reference.repos`, `dependencies.runtime.repos`, `dependencies.templates.repos` | commit | 已锁定上游引用 |
| `docs/initial-workspace-README.md`, `docs/sources.initial.lock.json`, `docs/reviews/*.md` | commit | 来源和历史评审/验证记录；不删除失败历史 |
| `integration/benchmarks/manifests/*.json` | commit | 冻结评测成员与门槛 |
| `integration/config/*`, `integration/scripts/*`, `integration/tests/*` | commit | owned配置、实现和测试；C2修复前不能提交 |
| `scripts/build.sh`, `scripts/check_dependencies.py`, `scripts/fetch_dependencies.sh` | commit | 现有构建和锁定依赖工具 |
| `src/jt16_bringup/**` | commit | package/CMake、配置、插件导出、原生实现与测试 |
| `external/`, `build/`, `install/`, `log/`, `integration/results/`, `integration/models/`, `integration/worlds/`, `integration/benchmarks/generated/` | gitignore（已落实） | 非脏路径；不提交数据、生成物或运行产物 |

## 代码修改建议

| ID | 必须/建议 | 文件:行 | 函数或符号 | 改法 |
|---|---|---|---|---|
| C2 | 必须 | `integration/scripts/run_barn_navigation.sh:22` | 缺失NAV_GOAL前置检查 | 失败JSON保留，退出非零；加缺失mapping-result分支测试 |

### C2 补丁示意

- 目标: `integration/scripts/run_barn_navigation.sh` 缺失mapping-result分支。
- 动作: 替换退出码，不改结果类型或门槛。

```diff
   # 写 MISSING_LIO_ODOMETRY / scored_success:false
-  exit 0
+  exit 2
```

测试使用临时map文件和缺失/无效mapping-result，执行该前置分支或真实入口（不启动Gazebo），断言返回非零且JSON仍是 `MISSING_LIO_ODOMETRY`、`scored_success:false`。复跑50UT及新增测试；不要为这个入口修复重跑全量benchmark。

## Review comments

| ID | 维 | 等级 | 挡住提交 | 位置 | 标题 |
|---|---|---|---|---|---|
| C1 | D1 | BLOCK | 1 | `integration/results/v0/timed-mppi-motion-memory-pilot` | 同版本最终六例证据未齐 |
| C2 | D4 | BLOCK | 1 | `integration/scripts/run_barn_navigation.sh:22` | 缺失建图失败却退出成功 |

### C1. 同版本最终六例证据未齐

- 维: D1 交付
- 等级: BLOCK
- 挡住提交: 1
- 位置: `integration/results/v0/timed-mppi-motion-memory-pilot`
- 问题: 当前单例pilot不能证明两套冻结小集同一最终source/runtime各3/3。
- 证据: 调用未提供最终六例产物；live pilot未完成最终门槛，旧weight pilot/repeat为不同源码。
- 代码修改: 无
- 建议: 单例通过后冻结新输出根，Arena三例、BARN三例重新建图及导航；任一失败先修。交付JSON/NPZ、mapping结果、TF审计、source/runtime/input身份核对和离线重评分，以及两套严格3/3聚合。不要启动全量评测。
- 处置: 补证据

### C2. 缺失建图失败却退出成功

- 维: D4 质量
- 等级: BLOCK
- 挡住提交: 1
- 位置: `integration/scripts/run_barn_navigation.sh:22`
- 问题: 无有效lio_final_xyyaw时返回0，上层 `set -e`/自动执行器误判该任务成功。
- 证据: 同分支明确写 `result:MISSING_LIO_ODOMETRY`、`scored_success:false`，然后 `exit 0`；正常导航尾部才返回严格scoring退出码。
- 代码修改: 见「代码修改建议」C2
- 建议: 改非零并补前置失败测试；保留原JSON和失败历史。
- 处置: 改代码

未运行新仿真、全量benchmark、硬件和ROS smoke，不改动live进程；只读取既有原生测试/build证据。
风险：本报告不能作为最终小集通过证明；必须补C1后再独立复审最终交付状态。
