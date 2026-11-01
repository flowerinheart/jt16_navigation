# V0 · LiDAR-only 仿真基线

状态：完成。Git 标签：`v0`。日期：2026-11-01。

交付：官方 Hesai FAST-LIO 建图/里程计、二维地图保存、AMCL 定位、Nav2
MPPI 导航、实时 LiDAR costmap、轻量运动估计与时间相关评分。不含识别模型。

验收：同版 Arena Quick3 **3/3**，重新建图 BARN Quick3 **3/3**；六例
严格成功、零碰撞、零输入失败、定位/评分有效。三例建图 COMPLETE。
54 项 Python、5 项原生测试通过；独立 delivery 六维 PASS，Ponytail Ship。

证据：`integration/results/v0/timed-mppi-startup-pilot/`。原始结果留在本机，
不提交依赖/构建目录/NPZ/PCD；技术报告附指标、来源哈希及数据回放。

边界：V0 完成的是小规模仿真开发基线，不是完整 benchmark 或实机验收。
BARN22 距 NAV150 仅余 0.696s；曾出现 DDS 启动应答超时。安装标定、
真实噪声/轮动态、遮挡/转弯及恢复动作安全未获实机证明。

评审：[delivery](../reviews/20261101-current-gate-rereview.md) ·
[Ponytail](../reviews/20261101-current-gate-ponytail-review.md)。
