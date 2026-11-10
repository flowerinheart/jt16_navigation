# 项目文件分类与输出路径

默认工作目录为 `~/.jt16-navigation/`。仅用 `JT16_DATA_HOME` 改变根目录；调用者显式提供的输出路径优先。

| 类型 | 内容 | 位置 | Git |
|---|---|---|---|
| 核心工程 | src、运行配置、launch、通用脚本、测试、固定依赖清单和补丁 | 仓库 | 提交 |
| 规范与简洁结论 | AGENTS.md、需求/设计约定、简洁验收状态 | 仓库 docs | 提交，按用途判断，不按作者判断 |
| 运行结果 | 原始评测、轨迹、点云、日志、资源测量 | evaluations/版本/批次 | 不提交 |
| 生成评测报告 | comparison.json、failures.csv、审计/测试回执、生成 HTML 和配图 | reports/evaluation/版本/批次 | 不提交 |
| AI 工作报告 | 长篇 review、诊断过程和一次性分析报告 | reports/ai/日期 | 不提交；经整理成为设计规范的文档除外 |
| 实验材料 | 单次诊断脚本、候选配置、详细进度 | experiments/日期或批次 | 不提交；通用工具和正式回归测试迁回源码仓 |
| Demo | 最终图片、视频、来源与哈希索引 | demos/版本 | 不提交；渲染代码提交 |
| 中间缓存 | colcon build/install/log、生成模型和场景、可重建文件 | cache/ | 不提交 |
| 短期 scratch | 临时抽帧、统计和编译验证 | /tmp | 不提交，允许清理 |

入口默认值由 `integration/scripts/artifact_paths.py` 统一提供：

- `run_development_set.py`、`run_nav2_goal.py`：默认生成新的带时间戳 evaluations 子目录。
- 单例 BARN/Arena、建图、FAST-LIO/projection 脚本：可省略输出参数，保存到 evaluations；全量批次脚本沿用版本/数据集目录及已有续跑规则。
- V0 `render_report.py`：生成 HTML、JSON 和配图进入带时间戳的 reports/evaluation 目录，报告正文从仓库读取。
- V1 `compare_results.py`、`render_report.py`：省略 `--output` 时生成新的 reports 子目录。
- `demos/render_samples.py`：媒体及索引默认写入 demos，读取 evaluations 中的原始批次。
- build 和场景生成入口为缺失的原生缓存路径创建忽略的兼容链接，实际写入 cache；既有真实目录不自动覆盖，需迁移时显式处理。
- `--output` 为必填的叶子转换/打分工具继续由上层传入批次路径，不另建隐式输出，避免结果散落。

仓库旧路径的本机软链接仅用于兼容读取，不随 Git 提交。历史记录中的路径与哈希不改写；跨机器要复制工作目录并重建链接。缓存可重新生成，运行结果和最终 Demo 必须备份。`migration-20261109.json` 与 `classification-migration-20261109.json` 保存迁移记录。

完整生成数据已明确取消 Git 跟踪；这会产生删除旧已跟踪产物的 Git 变更，不代表删除本地评测数据。简洁报告保留验收结论，不为缩小 diff 删除评分代码或测试。

容器 setup 显式设置 `JT16_DATA_HOME=/workspace/.jt16-navigation`，数据和缓存留在已有 `/workspace` Docker 卷中；打开容器时保留该环境变量。不要把持久产物写进一次性容器的未挂载 home。

新 checkout 的六类工作缓存按 checkout 绝对路径的稳定摘要隔离到 `cache/workspaces/<checkout-key>/`，避免不同 clone 共享 CMake/模型缓存。已有真实目录和历史兼容链接保持不变，不自动搬迁。

## 持续检查

`python3 scripts/check_repository_cleanliness.py` 用 `.gitignore` 检查当前 Git 索引，即使使用 `git add -f` 加入本地产物也会失败。GitHub push/PR 执行相同检查；是否作为合并必需检查由仓库分支保护设置决定。它不修改文件或索引，不改变已有 Git hooks。

`docs/reviews/`、详细 progress/checkpoint 和部署原始 JSON 已完整归档并取消跟踪；简洁部署结论仍在 `docs/deployment-evidence/`。迁移清单及 SHA-256 存在 `reports/ai/<日期批次>/repository-cleanup/migration.json`。共享 AI 规则可提交，会话状态和凭据按明确路径忽略。源码目录允许存在被忽略的 external/build/install/log 本地目录和兼容链接。
