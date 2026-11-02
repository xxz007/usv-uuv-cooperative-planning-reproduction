# MOOS与VRX平台

P5已完成本人操作／解释与工程独立核查。MOOS验证单艇任务闭环，VRX验证单WAM-V原例与独立二维对照；各运行分别启动和结束。原始数据中的角色按实际来源保留，结构整理的工程检查不计为新增本人运行。

## 不启动平台，重查保存数据

主环境准备后，在仓库根使用新输出目录：

```bash
.venv/bin/python -B tools/platforms/verify_moos_saved.py --source-run "$PWD/results/p5_platforms/raw/moos" --analysis-id saved_moos_01
.venv/bin/python -B tools/platforms/verify_vrx_original_saved.py --source-run "$PWD/results/p5_platforms/raw/vrx_original/observations" --output "$PWD/results/p5_platforms/checks/saved_original_01"
.venv/bin/python -B tools/platforms/verify_vrx_saved.py --source-run "$PWD/results/p5_platforms/raw/vrx_pair" --output "$PWD/results/p5_platforms/checks/saved_pair_01"
```

输出关注status、完成时间和实际位移／横偏。检查器保留控制器、配置、原例记录器和平台包装器的原SHA约束，不把目录迁移当成修改历史来源的理由。MOOS的TIMEOUT与非法目标REJECTED仍要求不误报完成。

## 从保存CSV重绘VRX配对

```bash
.venv/bin/python -B -m usv_uuc_cooperation.visualization.p5_vrx --source-run "$PWD/results/p5_platforms/raw/vrx_pair" --output "$PWD/results/p5_platforms/showcase/saved_plot_01"
```

该命令只读取保存数据，不重跑模型／仿真。原图见[平台结果](../results/p5_platforms/README.md)。

## 新平台运行

MOOS需要外部已编译的MOOS-IvP检出，按实际位置调整`--moos-root`：

```bash
.venv/bin/python -B -m usv_uuc_cooperation.experiments.p5_moos --run-id moos_new_01 --moos-root ../moos-ivp --role learner
```

VRX在兼容Linux中使用系统Python3.12、ROS2 Jazzy／Gazebo Harmonic，固定VRX pin与Fuel模型身份；详细准备和统一入口见[Linux环境与运行](vrx_environment.md)。原控制器／平台包装器与V02原例记录器字节沿用，以保留来源链。

任务发布成功只证明消息发出；MOOS完成还要求同任务ID反馈及实际NAV进入到达范围。VRX中推力发布、真实位移、实收图像分别核查。原例3秒零推力＋8秒左右各100N，配对为前方20米、标称1.5m/s、半径2.5米／保持1秒。

范围保持：没有新多艇平台协同、水下六自由度、真实声学或电池功率验证。[模型与研究缺项](scope.md)
