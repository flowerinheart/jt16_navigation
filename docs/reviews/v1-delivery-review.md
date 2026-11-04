# 六维交付评审

## 总结

| 项 | 值 |
|---|---|
| **可提交** | **0** |
| **整体等级** | BLOCK |
| 一句话 | 开发与复跑通过，全量未完成，交付不可提交 |
| D1 交付 | BLOCK |
| D2 验证 | PASS |
| D3 卫生 | NITS |
| D4 质量 | PASS |
| D5 设计 | PASS |
| D6 平台 | PASS |
| 挡住提交 | C1 |
| 代码修改 | 必须 0 处；建议 0 处（见「代码修改建议」） |
| 评论数 | BLOCK=1；NITS=1 |
| 下一步 | 先处理挡住提交的 C1，完整冻结评测结束后刷新证据并重新评审 |
| Goal complete | 0 |

仓库: `/home/haojiema/code/amd_robot/ref_arch/AMD_X100_Robotics_DevKit_1.0.3/lidar_slam/jt16_navigation`
范围: uncommitted
终态: 通用修复实现V1；Dev16严格16/16及10新图有效；指定两例各3次严格成功；冻结30地图、150 BARN及60 Arena完整，BARN≥105、Arena≥42、Arena碰撞≤6、有效地图30/30、PROCESS_FAILURE≤4/210，原始记录、身份、报告图与回放可核验。
未执行: 未 commit、未 push、未改代码；仅写本评审文件。

只读快照时间: **2026-11-04T03:28:38+00:00**。评测仍在串行写入，本结论仅覆盖该时刻。

## 六维打分

| 维 | 等级 | 命中的门槛 | 证据（路径/命令/exit/符号） |
|---|---|---|---|
| D1 交付 | BLOCK | 终态未发生；必交完整批次未跑完 | `comparison.json` 的 `v1.complete=false`、`performance_targets_met=false`；实时计数 BARN 111/150、严格成功 109，Arena 0/60，地图30/30有效。Dev16与六次复跑已达标，不能替代完整批次 |
| D2 验证 | PASS | 指定测试绿、覆盖新行为、golden未放宽 | 本评审执行 `/usr/bin/python3 -m unittest discover -s integration/tests -p 'test_*.py'`：66 tests、exit 0；`/usr/bin/python3 -m unittest discover -s docs/v1 -p 'test_*.py'`：2 tests、exit 0。指定XML为prediction 3/3、timed critic 4/4；XML及test-evidence源哈希匹配 |
| D3 卫生 | NITS | 无密钥，但仍有应gitignore的脏文件 | 下表逐路径三分；4个未跟踪生成图/回放按技能归入gitignore，保留本地及其哈希证据。新增依赖均为现有ROS平台包，运行库身份已记录，未新增浮动下载；diff未发现密钥 |
| D4 质量 | PASS | delta无正确性BLOCK，无新函数CCN≥15或nloc跨80，无两个屎山信号 | lizard覆盖14个实现文件及4个测试文件，无门槛超限；最高相关函数 `MotionPredictor.update` CCN14、`track_velocity` CCN13。radon CC显示同值，MI未输出B及以下；jscpd未安装，未运行，人工未发现≥10行新逻辑克隆。只判本次增量 |
| D5 设计 | PASS | 同层同目录同模式，复用现有模块/调用约定 | `run_development_set.py` 调用既有mapping/BARN/Arena runners；`compare_results.py`复用`score_v0.py`；`render_report.py`复用`docs/v0/render_report.py::map_panel`和`navigation_metrics.local_trajectory`。Smac/MPPI/BT沿用Nav2现有扩展点 |
| D6 平台 | PASS | 无一次功能新增框架抽象；现有扩展点及目标要求的评测生命周期 | 改动既有 `PredictionLayer::receive/updateBounds/updateCosts`、新增内部`transformPoints`，未新建Base/ABC/Protocol/registry/hook。新报告及Dev16 CLI入口、mapping进程组cleanup属于终态要求的评测编排；未新增框架或agent loop |

Dev16清单绑定V0原始结果及SHA256；原始16份`nav2-goal.json`均为SUCCEEDED、scored_success/scoring_valid/localization_success为true、≤150s、无碰撞/输入失败。10份新图均有效。`repeated-g/repeat-summary.json`及六份原始结果确认map2 obs5 seed0、warehouse obs10 seed0各3次严格成功。

独立重算当前与`v0`的grading fingerprint相同：`c91cff084951e9b63d2de29124ac85872a1c3a694c975233ddf09fd3055911c9`。检查现有173份run-provenance（Dev16 26、repeat 6、full 141）：当前源、runtime、输入一致；执行归档源及输入哈希全部一致。`run-identity-audit.json`旧快照仅171/273、完整性false，不能宣称最终身份集齐全。`protocol-audit.json`确认已存在案例的固定输入匹配V0，最终剩余案例仍需核验。

已查看三张指定PNG。navigation图展示Dev16对应V0全量案例；mapping图展示Dev16新图及V0 BARN22/144/277，V0 277明确显示无保存地图。图中三个Arena的G时间/间隙与Dev16原始记录对应。`development-evidence.json`14项SHA256全部匹配，包含图、poster、视频及三个案例原始轨迹/结果/身份。`ffprobe`确认回放14.25s、174693字节；渲染代码使用实际footprint、观测障碍、时间戳≤0.25s匹配，8倍速；poster标注Dev16 telemetry replay。未把这些开发集视觉当作全量结果。

ROS smoke仅读取`test-evidence.json`记录（Cyclone DDS、exit 0）和既有独立代码评审；未重新启动ROS/构建/install。该JSON没有独立构建日志字段；已有C++结果及源/runtime核验支持已安装实现身份，本轮不补做构建。

## Git 三分

| 路径 | 处置 | 理由 |
|---|---|---|
| `README.md` | commit | 交付评审与实现证据文档 |
| `docs/v1/plan.md` | commit | 交付评审与实现证据文档 |
| `integration/scripts/benchmark_geometry.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/build_mppi_parameters.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/follow_barn_path.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/motion_prediction.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/predict_obstacles.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/run_arena_navigation.sh` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/run_barn30_mapping.sh` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/run_barn_mapping.sh` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/run_barn_navigation.sh` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/run_nav2_goal.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/run_provenance.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/score_v0.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/scripts/write_map_bundle_metadata.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/tests/test_motion_prediction.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `integration/tests/test_regressions.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/CMakeLists.txt` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/config/nav2_params.yaml` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/include/jt16_bringup/prediction_layer.hpp` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/package.xml` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/src/prediction_layer.cpp` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/src/timed_obstacle_critic.cpp` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/test/prediction_layer_test.cpp` | commit | 实现终态所需源码、配置、脚本或回归 |
| `docs/reviews/v1-ponytail-review.md` | commit | 交付评审与实现证据文档 |
| `docs/v1/assets/arena-replay.mp4` | gitignore | 生成的展示图/回放，保留本地与SHA256记录 |
| `docs/v1/assets/comparison.json` | commit | 交付评审与实现证据文档 |
| `docs/v1/assets/development-evidence.json` | commit | 交付评审与实现证据文档 |
| `docs/v1/assets/failures.csv` | commit | 交付评审与实现证据文档 |
| `docs/v1/assets/mapping-comparison.png` | gitignore | 生成的展示图/回放，保留本地与SHA256记录 |
| `docs/v1/assets/navigation-comparison.png` | gitignore | 生成的展示图/回放，保留本地与SHA256记录 |
| `docs/v1/assets/protocol-audit.json` | commit | 交付评审与实现证据文档 |
| `docs/v1/assets/replay-poster.png` | gitignore | 生成的展示图/回放，保留本地与SHA256记录 |
| `docs/v1/assets/run-identity-audit.json` | commit | 交付评审与实现证据文档 |
| `docs/v1/assets/test-evidence.json` | commit | 交付评审与实现证据文档 |
| `docs/v1/compare_results.py` | commit | 交付评审与实现证据文档 |
| `docs/v1/progress.md` | commit | 交付评审与实现证据文档 |
| `docs/v1/render_report.py` | commit | 交付评审与实现证据文档 |
| `docs/v1/report.md` | commit | 交付评审与实现证据文档 |
| `docs/v1/test_compare_results.py` | commit | 交付评审与实现证据文档 |
| `integration/scripts/run_development_set.py` | commit | 实现终态所需源码、配置、脚本或回归 |
| `src/jt16_bringup/config/navigate_with_replanning.xml` | commit | 实现终态所需源码、配置、脚本或回归 |
| `docs/reviews/v1-delivery-review.md` | commit | 本次唯一交付评审文件 |

上表commit仅为路径归属，不代表当前获准提交；C1未关闭前不得commit。`integration/results/`、build/install/log及Python缓存已被现有.gitignore忽略，不应纳入提交。原始评测继续保留本地，不删除，不公开上传。

## 代码修改建议

无代码修改（本轮是补证据 / Git 三分）。

## Review comments

| ID | 维 | 等级 | 挡住提交 | 位置 | 标题 |
|---|---|---|---|---|---|
| C1 | D1 | BLOCK | 1 | `docs/v1/assets/comparison.json` | 完整冻结评测尚未完成 |
| C2 | D3 | NITS | 0 | `git:docs/v1/assets/arena-replay.mp4` | 生成展示媒体尚未归入忽略规则 |

### C1. 完整冻结评测尚未完成

- 维: D1 交付
- 等级: BLOCK
- 挡住提交: 1
- 位置: `docs/v1/assets/comparison.json`
- 问题: 30图已有效，但150 BARN与60 Arena尚未完整，Arena成功/碰撞及总PROCESS_FAILURE目标不能终验。
- 证据: report/comparison快照为BARN107/150（105成功）、Arena0/60；comparison完整性和全部五项pass均false；evaluation-request仍LAUNCHED。报告坦诚标注in progress，未发现虚报，但终态仍未发生。
- 代码修改: 无
- 建议: 让当前冻结串行评测跑完，不中途改参数、不重试挑结果；完整后执行 `/usr/bin/python3 docs/v1/render_report.py --full integration/results/v1/full-benchmark-20261104-g --dev16 integration/results/v1/dev16-candidate-g --output docs/v1/assets`，刷新report、comparison、failures及最终identity/protocol/test/development证据；确认30+150+60原始记录完整、五项目标全部通过、完整273身份可核验，再运行独立delivery-review。不得提前commit或标Goal complete。
- 处置: 补证据

### C2. 生成展示媒体尚未归入忽略规则

- 维: D3 卫生
- 等级: NITS
- 挡住提交: 0
- 位置: `git:docs/v1/assets/arena-replay.mp4`
- 问题: 四个未跟踪生成图/回放仍出现在待提交集合。
- 证据: git status列出navigation-comparison.png、mapping-comparison.png、replay-poster.png、arena-replay.mp4；现有.gitignore未覆盖这些路径；delivery-review技能将未跟踪demo音视频图归入gitignore。
- 代码修改: 无
- 建议: 在.gitignore增加这四个精确路径，保留本地文件及development-evidence.json中的来源/SHA256；不要删除原始或媒体证据，不公开上传。本报告仅建议三分，未修改.gitignore。
- 处置: gitignore

未运行jscpd、ROS、仿真、构建/install或硬件验证；全量评测正在继续，以上计数是快照，尚不能证明V1放行或独立泛化。
