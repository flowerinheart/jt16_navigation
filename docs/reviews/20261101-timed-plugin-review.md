# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **0** |
| **整体等级** | BLOCK |
| 一句话 | 小集终态未达成，单例到达但碰撞评分无效 |
| D1 交付 | BLOCK |
| D2 验证 | PASS |
| D3 卫生 | PASS |
| D4 质量 | NITS |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | C1 |
| 代码修改 | 必须 0 处；建议 1 处（见「代码修改建议」） |
| 评论数 | BLOCK=1；NITS=1 |
| 下一步 | 先处理 C1：修复采集缺口后冻结版本，补齐两套小集 |
| Goal complete | 0 |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`
范围: uncommitted；全部未跟踪 owned source 作为新代码，重点为时间相关 MPPI critic 及连接路径。
终态: 同一最终源码/runtime 的 Arena Quick3、BARN Quick3 各 strict 3/3，零碰撞/进程失败，映射、传感、评分和 provenance 可信。
需求: `plan.md` 与调用单；允许 LiDAR-only 运动估计和原生 MPPI 时间评分插件。
范围外: 全量 benchmark、实机标定、commit/push、调整场景/种子/速度/NAV150/评分。
未执行: 未 commit、未 push、未改实现代码；未构建/重装或打断正在运行的仿真。

## 六维打分

| 维 | 等级 | 命中的门槛 | 证据 |
|---|---|---|---|
| D1 交付 | BLOCK | 终态未发生；必交产物未齐或不达通过标准 | `timed-mppi-pilot.5H9WXx/arena/map2_obs05/seed_00/nav2-goal.json` 为 action SUCCEEDED、107.388s，但 scoring_valid=false、scored_success=false；最终双 Quick3 根目录尚未提供 |
| D2 验证 | PASS | 指定 UT 绿且覆盖本次行为，未放宽 golden | 独立运行 46 UT，exit 0；读取 native gtest XML 共 4 tests/0 failures/0 errors，其中新 critic 3 tests，覆盖 pluginlib 加载、时间匹配、姿态、空输入和失效输入 |
| D3 卫生 | PASS | 路径已三分，无发现的密钥或未钉死新依赖 | Git 仅有未跟踪顶层项；三类 dependencies repos 均使用固定 revision；大型结果/外部仓库/build/install/log 已 ignore |
| D4 质量 | NITS | 单点小乱；无已证实 correctness BLOCK | C2 命令退出状态仍基于 action 而不是 strict score。lizard 对全部 owned scripts/tests/C++ 无门槛告警；radon CC 最大14；jscpd 不在 PATH，未运行 |
| D5 设计 | PASS | 使用现有基类/调用约定，同层同模式 | `TimedObstacleCritic : mppi::critics::CriticFunction`、pluginlib 导出，位于现有 prediction layer 同包；publisher 复用已有 MotionPredictor 和 scan/FAST-LIO odom |
| D6 平台 | PASS | 新扩展点获得明确授权 | 新 `TimedObstacleCritic` / plugin XML / subscription 回调和 `score`；调用单明确允许原生轻量时间评分插件，不替换 MPPI 框架 |

验证命令：

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s integration/tests -q
lizard -w -T cyclomatic_complexity=14 -T nloc=80 -T parameter_count=8 integration/scripts/*.py integration/tests/*.py scripts/*.py src/jt16_bringup/src/*.cpp src/jt16_bringup/test/*.cpp
radon cc -s -n C --no-assert integration/scripts/*.py integration/tests/*.py scripts/*.py
radon mi -s -n B integration/scripts/*.py integration/tests/*.py scripts/*.py
```

独立检查了 `motion_prediction.py`、`predict_obstacles.py`、原生 critic/header/注册/CMake/package、config generator、runner、provenance、sensor/score 几何与相关测试、Arena 调用脚本和 sibling prediction layer。当前实现按 scan 时间发布实测表面与速度，critic 使用 pose/scan 时间差加 `(j+1)*model_dt`；真实障碍 CostCritic 保留，静态未来扫掠层在 MPPI 配置关闭。未发现足够证据支持新增 critic 的逻辑 BLOCK。

已独立从该 pilot 的 trajectories NPZ 重放 `score_navigation`，同样得到 `observation timestamp gap` 和 strict false。executed-inputs 的 source hash 全部吻合，runtime hash 与当前安装吻合；当前 `integration/scripts/sim_state_adapter.py` 已不同于 pilot 记录的 source hash，因此该 pilot 也不能充当当前最终源码的回归证明。TF audit 为 PASS，`map->odom` 单一 AMCL、`odom->base_link` 单一 LIO bridge。

Radon MI：runner 14.87，集中回归测试文件 2.55；后者没有可比较的 tracked 旧版本，不能据此声称本次 MI 下降，也不把测试集合的低 MI 等同产品逻辑 bug。installed smoke 未独立复跑；读取测试代码，不将其报告中的历史 PASS 冒充本轮执行。正在进行的 service diagnostic 尚未作为终态证据。

## Git 三分

| 路径 | 处置 | 理由 |
|---|---|---|
| `.gitignore` | commit | 已排除外部依赖、运行产物及缓存 |
| `LICENSE` | commit | owned source 许可 |
| `README.md` | commit | 项目入口说明，交付时应准确反映最终候选状态 |
| `dependencies.reference.repos` | commit | 固定参考依赖 |
| `dependencies.runtime.repos` | commit | 固定官方运行依赖 |
| `dependencies.templates.repos` | commit | 固定模板依赖 |
| `docs/` | commit | 来源锁、评审与验证记录；历史 BLOCK 应保留 |
| `integration/`（非 ignored 文件） | commit | 仿真采集/评分/测试/冻结 manifest 源码 |
| `plan.md` | commit | 需求和小集晋级规则 |
| `scripts/` | commit | 依赖和构建入口 |
| `src/` | commit | 配置、原生插件及测试 |
| `external/`, `build/`, `install/`, `log/`, `integration/results/`, `integration/benchmarks/generated/` 等既有 ignore 路径 | gitignore | 已生效，无待处理脏路径；不纳入提交源码 |

以上 commit 是路径分类，不是提交许可：C1 未解决，不得提交或标 complete。没有发现需 revert 的脏文件。

## 代码修改建议

| ID | 必须/建议 | 文件:行 | 函数或符号 | 改法 |
|---|---|---|---|---|
| C2 | 建议 | `integration/scripts/run_nav2_goal.py:187`；`integration/scripts/run_arena_navigation.sh:44` | terminal exit；Arena shell entry | 记录完整 JSON 后，用 strict success 产生退出状态，且外层保留该状态 |

### C2 补丁示意

- 目标: runner terminal exit 和 Arena shell entry。
- 动作: 替换判据、补传播。历史结果不改写，批量评分脚本仍可显式收集失败。

```diff
-raise SystemExit(0 if result==TaskResult.SUCCEEDED else 1)
+raise SystemExit(0 if summary['scored_success'] else 1)
```

```diff
 kill -USR1 "$PM" 2>/dev/null||true;wait "$PM" 2>/dev/null||true
+exit "$STATUS"
```

如采用，增加一个“action SUCCEEDED 但 scoring invalid”的 terminal-status 回归断言，并对 BARN 同类 entry 检查传播；不要更改 score 阈值。

## Review comments

| ID | 维 | 等级 | 挡住提交 | 位置 | 标题 |
|---|---|---|---|---|---|
| C1 | D1 | BLOCK | 1 | `integration/results/v0/timed-mppi-pilot.5H9WXx/arena/map2_obs05/seed_00/nav2-goal.json` | 到达不等于可信无碰撞成功，最终双小集未齐 |
| C2 | D4 | NITS | 0 | `integration/scripts/run_nav2_goal.py:187` | strict 失败仍可得到成功退出状态 |

### C1. 到达不等于可信无碰撞成功，最终双小集未齐

- 维: D1 交付
- 等级: BLOCK
- 挡住提交: 1
- 位置: `integration/results/v0/timed-mppi-pilot.5H9WXx/arena/map2_obs05/seed_00/nav2-goal.json`
- 问题: 单例到达但碰撞评估无效，最终同版本 Arena3+BARN3 尚不存在。
- 证据: action SUCCEEDED/107.388s，collision=null，scoring_valid=false，scoring_error=observation timestamp gap，scored_success=false；NPZ 独立重放一致，且 current adapter source 已变。
- 代码修改: 无
- 建议: 先用 sensor/GT 时间戳定位每个 actor 的采集缺口，不放宽 .25s 容差；修复后在新根目录重新采集 pilot，严格通过后冻结源码/runtime，执行同版本 Arena Quick3 和 BARN Quick3（后者 fresh mapping）各 3/3，重放 NPZ、核对身份和 TF 后再次评审。不要跑完整评估，不混用旧版本结果。
- 处置: 补证据

### C2. strict 失败仍可得到成功退出状态

- 维: D4 质量
- 等级: NITS
- 挡住提交: 0
- 位置: `integration/scripts/run_nav2_goal.py:187`
- 问题: 子进程退出码仅看 action，外层 Arena shell 收集 STATUS 后也不返回，命令级检查无法识别本次 strict 失败。
- 证据: 末行 `0 if result==TaskResult.SUCCEEDED else 1`；本 pilot 正好 action true/strict false；Arena entry 最后结束于 resource-monitor wait。JSON score_arena/score_quick3 使用 strict fields，未发现因此抬高实际评分，所以本项不是 metrics correctness BLOCK。
- 代码修改: 见「代码修改建议」C2
- 建议: 按补丁使单例 entry 返回 strict 结果；仍完整保存失败 JSON/NPZ，批量调用可显式忽略非零以继续收集，晋级不能仅看 exit 0。
- 处置: 改代码

未检查：未重跑构建、installed smoke、完整 benchmark、实机标定；本轮不修改实现或历史结果。
风险：常速度可见紧凑表面估计不是转弯/遮挡安全证明；目前无可信最终双小集成功证据。
