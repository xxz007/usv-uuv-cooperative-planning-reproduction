# P5 完整阶段验收与平台证据

**P5 于 2026-10-31 完整通过，工程 VERIFIED、展示 RENDERED_VERIFIED。** 原范围包含 MOOS 原例、分配器到 MOOS 的闭环、VRX 官方 WAM-V 原例及独立二维／VRX 同任务对照。本人 MOOS 一次主命令、VRX 一次 `all` 命令（两个独立子运行）及新增解释均完成；P6 未开始。[阶段验收 JSON](p5_acceptance.json)

| 三项综合验收 | 状态 |
|---|---|
| 平台原例与分配执行闭环 | PASS |
| 完成反馈、超时／非法目标与同任务对照 | PASS |
| 本人执行／解释、保存日志独立核查、展示及来源恢复 | PASS |

当前发布 FRAMEWORK-V20；MOOS 本人实际执行／复核为 V18，VRX 本人统一入口为 V20。教师与本人角色、脚本 SHA 和运行分别记录，不用教师检查替代本人证据。106 项源码／配置、4 项输入及本地完整阶段恢复包通过身份／独立恢复核查，保存日志重查没有新增模拟；完整恢复包与私人课堂回答不发布。

## 本人 MOOS 闭环

| 系统 | 完成任务 | 完成时间／模拟秒 | 最大横向偏差／m | 坐标包络违规 |
|---|---:|---:|---:|---:|
| MOOS 实际 NAV | 1/1 | 19.08 | 6.953660 | 0 |
| 独立二维运动学参考 | 1/1 | 13.75 | 0 | 0 |

共同起点 (0,-20) m、目标 (30,-20) m、标称速度 2 m/s、到达容差 2.5 m、初始航向元数据 0°。成功同时要求同一任务 ID 的派发、Helm 完成反馈和容差内的实际 NAV；短等待窗口 TIMEOUT 与发布前拒绝的非法目标均未误报完成。uQueryDB 等待窗从查询进程启动计算，不是严格的派发截止。二维参考不含 MOOS 转向／速度响应。

[本人核查](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人V01_保存日志复核_V01/result.json) · [配对指标](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人V01_保存日志复核_V01/paired_metrics.json) · [实际 NAV](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人V01_保存日志复核_V01/success/trajectory.csv) · [二维 CSV](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人V01_保存日志复核_V01/reference_2d.csv) · [来源 SHA](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人V01_保存日志复核_V01/source_SHA256.json)

MOOS 源码检出为 `dd7030bdba1ea2d01524f0ff2364a3ed0d2c56a6`，实际二进制 SHA 已保存；二进制对应构建提交为 **UNKNOWN**，不由源码检出推定。教师工程日志与图作为单独历史来源保留。

## 本人 VRX 官方原例

原例使用官方默认 WAM-V 和 `sydney_regatta`，不改原例船体。原生 Gazebo Pose_V 按 `Pose.name == wamv` 选取模型，保留 entity id、全局 header、原始世界位置／四元数；ROS 用于时钟、推力和图像。3 模拟秒零推力后，左右各 100 N、约 8 秒内实际水平位移 **6.093563 m**；实际收到 **1280×720、rgb8** 原始 Image 缓冲。

![本人原例实际相机帧](../输出文件/阶段结果/P5/P5-V01/showcase/P5_本人VRX_V01/original_first_camera_frame.png)

[原例独立核查](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人VRX_V01_原例独立核查_V01/checks.json) · [原始位置 CSV](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/original/observations/model_pose.csv) · [图像 header／SHA](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/original/observations/first_image_header.json) · [RGB 二进制](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/original/observations/first_image.data.bin) · [启动及清理记录](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/original/launch_metadata.json)

推力发布、真实位移和渲染帧分别记录。原始 `phase` 是处理阶段；最终分段按原生时间戳和实际发布边界重选半开区间，排除协议前／停止后的采样，不据处理顺序归段，也不以单个原例证明普遍动力学因果。

## 本人二维／VRX 同任务对照

| 系统 | 完成时间／模拟秒 | 最大横向偏差／m | 预定义包络违规事件 |
|---|---:|---:|---:|
| VRX 完整 WAM-V 动力学 | 15.7 | 0.008682373 | 0 |
| 独立理想二维运动学 | 12.7 | 约 0（5.46×10⁻¹³） | 0 |

同一实测起点／朝向、前方 20 m 目标、标称 1.5 m/s、半径 2.5 m、连续有效采样保持 1 s、预算 90 模拟秒。包络是冻结起点／朝向下纵向 [-5,30] m、横向 [-10,10] m。时间零点为 `dispatch_clock_s`；指标／图截止完成时刻，排除关停阶段，未完成时间为 null。

对照 URDF 在完整官方船体上只添加 OdometryPublisher；URDF／SDF 结构比较确认质量、惯量、H 推进、浮力、阻尼及 7 个传感器保留。二维参考独立执行，速度响应理想化，不含 WAM-V 动力学，不声称物理等价。

![本人同坐标任务轨迹](../输出文件/阶段结果/P5/P5-V01/showcase/P5_本人VRX_V01/2D_VRX_same_task.png)

[对照独立核查](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人VRX_V01_对照独立核查_V01/checks.json) · [指标 CSV](../输出文件/阶段结果/P5/P5-V01/checks/P5_本人VRX_V01_对照独立核查_V01/metrics.csv) · [共同输入](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/pair/observations/common_inputs.json) · [Odometry](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/pair/observations/odometry.csv) · [二维 CSV](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/pair/observations/reference_2d.csv) · [进度／误差图](../输出文件/阶段结果/P5/P5-V01/showcase/P5_本人VRX_V01/VRX_progress_error.png) · [绘图来源](../输出文件/阶段结果/P5/P5-V01/showcase/P5_本人VRX_V01/来源.json) · [三图视觉核查](../输出文件/阶段结果/P5/P5-V01/showcase/P5_本人VRX_V01/视觉核查_V01.json)

## 只重查保存日志

从仓库根运行，新核查目录不得已存在；这些命令不启动平台或重跑二维模型：

```bash
.venv/bin/python -B 课时资料/P5/源码/复核P5保存日志_V01.py --source-run "$PWD/输出文件/阶段结果/P5/P5-V01/checks/P5_本人V01_保存日志复核_V01" --analysis-id replay_p5_moos_saved_01
.venv/bin/python -B 课时资料/P5/源码/复核P5_VRX原例保存日志_V01.py --source-run "$PWD/输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/original/observations" --output "$PWD/输出文件/阶段结果/P5/P5-V01/checks/replay_p5_original_saved_01"
.venv/bin/python -B 课时资料/P5/源码/复核P5_VRX保存日志_V01.py --source-run "$PWD/输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/pair/observations" --output "$PWD/输出文件/阶段结果/P5/P5-V01/checks/replay_p5_pair_saved_01"
```

新运行见 [Linux 环境及统一入口](vrx_environment.md)。MOOS、VRX 和两次 VRX 子运行均独立启动／关停，不能拼接为统一航迹。0 包络违规不等于碰撞安全；不声明真实能耗、电池、实艇或水声验证。
