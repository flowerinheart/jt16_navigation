# Versioned evaluation demos

保存实际评测记录生成的建图图例与导航回放。每个版本单独存放，后续版本沿用相同目录结构。

```text
demos/
  v0/
    mapping/                 # BARN 建图图例
    navigation/success/      # 严格成功 MP4 与末帧 PNG
    navigation/failure/      # 严格失败 MP4 与末帧 PNG
  v1-candidate-g/             # 候选版本，尚未完成 V1 全量验收
    mapping/
    navigation/success/
    navigation/failure/
  samples.json               # 每项原始路径、结果、SHA256、视频规格
  render_samples.py           # 从保存记录重建；不启动仿真
```

## 建图图例

两个版本各选 BARN 22、144、255，共六张；展示保存的占用地图、采集轨迹与 FAST-LIO 轨迹，并标注 ATE 和占用格 F1。图例来自各版本的全量评测建图记录；不按成绩挑选三个最优地图。

| 世界 | V0 | V1 candidate G |
|---|---|---|
| BARN 22 | [PNG](v0/mapping/barn-022.png) | [PNG](v1-candidate-g/mapping/barn-022.png) |
| BARN 144 | [PNG](v0/mapping/barn-144.png) | [PNG](v1-candidate-g/mapping/barn-144.png) |
| BARN 255 | [PNG](v0/mapping/barn-255.png) | [PNG](v1-candidate-g/mapping/barn-255.png) |

## 导航成功与失败

全部来自实际执行过的全量评估案例，非开发集、非另跑演示。按原始 `scored_success` 分类；action 返回 SUCCEEDED 但发生碰撞仍归失败。

| 版本 | 案例 | 结果 | MP4 |
|---|---|---|---|
| V0 | Warehouse obs5 seed0 | 严格成功，94.508 s | [播放](v0/navigation/success/arena6-small_warehouse_obs05-seed_00.mp4) |
| V0 | Warehouse obs10 seed0 | action SUCCEEDED，但有碰撞，严格失败 | [播放](v0/navigation/failure/arena6-small_warehouse_obs10-seed_00.mp4) |
| V0 | Map2 obs10 seed0 | TIMEOUT，150.080 s | [播放](v0/navigation/failure/arena6-map2_obs10-seed_00.mp4) |
| V1 candidate G | BARN 22 seed0 | 严格成功，39.780 s | [播放](v1-candidate-g/navigation/success/barn30-world_022-navigation-seed_00.mp4) |
| V1 candidate G | BARN 111 seed3 | FAILED，28.280 s；日志连续 no valid path found | [播放](v1-candidate-g/navigation/failure/barn30-world_111-navigation-seed_03.mp4) |
| V1 candidate G | BARN 266 seed0 | TIMEOUT，150.098 s；最终距目标 0.442 m | [播放](v1-candidate-g/navigation/failure/barn30-world_266-navigation-seed_00.mp4) |

视频为 **8 倍速遥测回放**：保存的真实仿真执行轨迹、实际 footprint、地图及障碍观测；不是相机录像或实机视频。障碍观测距当前时刻超过 0.25 s 时隐藏，不推测位置。标题显示完整案例、action 结果、严格结果、碰撞标志和时间；同名 PNG 是末帧。

V0 来源：`integration/results/v0/full-benchmark-20261101/`。
V1 candidate G 来源：`integration/results/v1/full-benchmark-20261104-g/`。该批次停于时间限制，仅执行 133/150 BARN，Arena 未开始；这些样本不代表 V1 已验收，也不构成版本成功率比较。

## 重建与核验

在仓库根目录执行 `/usr/bin/python3 demos/render_samples.py`。复用现有绘图与坐标转换工具，读取保存记录，不运行导航，不覆盖原始评测数据。生成六张地图、六个 MP4、六张视频末帧及来源清单；脚本检查成功分类、有限坐标、H.264 编码和播放时长。

[samples.json](samples.json) 记录每项原始结果与文件 SHA256。[validation.json](validation.json) 记录 12 项来源和输出哈希、执行归档输入、六个完整视频解码及末帧校验通过。生成媒体保留本地，源码、说明和清单可单独版本管理。

## 第一人称 3D 双视角回放

`-first-person.mp4` 同时显示左侧俯视轨迹和右侧第一人称透视图，时间同步、8 倍速。右图从原始场景 SDF/world 中的墙体、圆柱尺寸及世界坐标 GT 重建，移动圆柱使用同一时刻的障碍记录。虚拟相机水平朝向机器人前方，高度 0.38 m、水平视场 90°；这是可视化相机，不代表硬件 RGB 相机标定。保留原始 2D 视频和评测记录。

渲染使用 NumPy CPU 光线与立体几何求交，包含地面、遮挡和距离明暗；不启动 Gazebo，不需要 GPU。支持当前基准的直立盒体和圆柱，不还原 Gazebo 的光照、阴影、机器人网格或纹理。缺失动态观测仍隐藏，不补造运动。

```sh
python demos/first_person.py  # 摄像机朝向与遮挡自检
python demos/render_samples.py --case arena6/map5_obs10/seed_00 --first-person
```

依赖沿用现有报告渲染环境（NumPy、Matplotlib、Markdown、FFmpeg）。四个选定成功案例的双视角视频及来源/场景哈希见 [selected-first-person-20261109.json](selected-first-person-20261109.json)。

## 输出目录

渲染代码保留在仓库；媒体、来源索引和校验清单默认写到 `~/.jt16-navigation/demos/`，可通过 `JT16_DATA_HOME` 改根目录。原始批次从 evaluations 读取。本文旧相对链接在本机通过忽略的兼容链接可读，Git clone 不包含这些媒体；目录规则见 [artifact-storage.md](../docs/artifact-storage.md)。
