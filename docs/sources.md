# 软件、平台与资产来源

| 组件 | 来源与版本身份 | 本仓库处理 |
|---|---|---|
| PythonRobotics | [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)，原参考检出 `cdd0cc888802b584c2d654ca85e3c5460973487d`，MIT | 记录参考，不包含完整第三方检出 |
| MOOS-IvP | [moos-ivp/moos-ivp](https://github.com/moos-ivp/moos-ivp)，源码检出 `dd7030bdba1ea2d01524f0ff2364a3ed0d2c56a6`，组件许可见上游 COPYING | 外部构建；实际二进制 SHA 已保存，binary build commit 为 UNKNOWN，不能用源码 HEAD 代替 |
| Pyomo | [官方文档](https://pyomo.readthedocs.io/)，原隔离环境 6.10.1 | 声明依赖，调用真实求解接口 |
| HiGHS | [官方项目](https://highs.dev/)，原环境 highspy 1.15.1 | 独立求解依赖 |
| OR-Tools | [官方文档](https://developers.google.com/optimization)，原环境 9.15.6755 | CP-SAT 分配；整数缩放目标与浮点路线分列 |
| VRX | [固定提交](https://github.com/osrf/vrx/tree/7609d1bd90ce7edb29d040a082f949e8b089c864)，jazzy | 官方默认原例实际运行；对照完整船体只加 GT Odometry，URDF／SDF 结构比较 PASS；第三方检出不发布 |
| ROS／Gazebo | [ROS Jazzy](https://docs.ros.org/en/jazzy/)、[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)；实测 gz-sim 8.15.0、ros_gz 1.0.24 | Linux 外部运行环境；[详细包版本](vrx_runtime_versions.json) |
| 原生接收 | [gz-transport13](https://github.com/gazebosim/gz-transport/tree/gz-transport13)、[gz-msgs10](https://github.com/gazebosim/gz-msgs/tree/gz-msgs10) | 原例按 Pose.name／id 接收世界 Pose_V；ROS 另收时钟和图像，发布左右推力 |
| Fuel 模型 | [官方 Fuel](https://app.gazebosim.org/)、fuel-tools9 9.1.1；13 个实际版本见[环境说明](vrx_environment.md) | 保存[117 文件 SHA 身份](../输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_pair_teacher_V01/environment/fuel_cache_identity_V04.json)，不随仓库发布大型模型；模型许可见各自 metadata |

VRX Git pin 与 Fuel 资产身份分别记录。未指定版本的 Fuel URL／tip 会随服务端变化；模型目录名或 model.config 的版本字段不能独自证明服务端资源身份。Sydney3／chair3 使用已记录官方响应版本及 ZIP SHA 的原始 archive 导入，其他模型保存解析目录及文件 SHA；不把代码 pin 说成资产也已固定。

原始 D* 按 Stentz 原算法 h／p／k、OPEN 与回指针机制适配，保留实现说明差异，不声称逐字复制伪代码或实现 Focussed D*。扫描作用于任务访问顺序，单段使用 A*，不是区域全格覆盖。PSO 保留随机预算、路径表示和失败状态，不用精确搜索自动修补结果。

本项目尚未选择独立的开源许可证。第三方软件与模型分别受其许可约束，上表不为本项目授予新的公开使用许可。仓库保持 PRIVATE；私人课堂回答、连接信息、完整恢复包和第三方大型资产不在交付范围。P5 的实际证据和适用边界见 [P5 平台页](p5_platform.md)。
