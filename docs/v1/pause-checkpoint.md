# Paused checkpoint — 2026-11-04 12:50 CST

用户要求暂停，等待明确恢复通知。候选G源码/参数/运行库保持冻结；串行执行，150s仿真评分期限不变。

| 阶段 | 已完成 | 待完成 |
|---|---|---|
| 建图 | 30/30有效 | 0 |
| BARN原始导航 | 150/150；144严格成功，2 FAILED、3 TIMEOUT、1 PROCESS_FAILURE；0碰撞 | 0 |
| Arena原始导航 | 10/60；10严格成功，0碰撞/进程失败 | 50（含当前冻结案例） |
| 超时独立复跑 | 尚未开始 | 当前3个BARN TIMEOUT，后续新增TIMEOUT各一次 |

当前冻结案例：`arena6/small_warehouse_obs10/seed_00`。13个所属进程组、36个进程全部停止；其他任务未操作。

主批次：`integration/results/v1/full-benchmark-20261104-g/`。
暂停状态与精确PID、PGID、启动ticks：`orchestration/user-pause.json`。
编排：`/tmp/jt16-v1-complete-g.py`；归档副本`orchestration/complete_missing_and_retry_timeouts-attempt2.py`。
执行会话28207，根PID919522；此前原始133例的结果/轨迹/身份哈希再次核验未变化。

恢复时先核对暂停清单中PID和启动ticks是否仍匹配，再只向记录所属进程组发送SIGCONT。保留进程时不重新启动批处理，以免重复运行当前案例。墙钟看门狗期限在SIGSTOP期间仍流逝，恢复时可能触发；保留真实结果，按用户规则仅TIMEOUT另复跑一次，不改评分或挑结果。

Arena完成后编排会在`integration/results/v1/timeout-retries-g-20261104/`各重跑一次原TIMEOUT，原主表不替换；新超时也只一次，不无限重试。补齐主210记录和复跑后刷新comparison、身份/协议审计、报告与demo状态，重新独立delivery-review。当前D1完整性BLOCK未关闭，未commit/tag/push。
