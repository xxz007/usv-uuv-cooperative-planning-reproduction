# USV／UUV 协同任务规划与仿真复现

这个项目从地图与路径搜索出发，逐步接入任务分配、访问顺序、通信中继、几何能耗、洋流和动态任务事件，形成 1 USV＋5 UUV 的二维协同执行案例。项目保存可运行源码、固定输入、代表结果和版本校验，便于复现方法及检查结果来源。

当前发布源码为 **FRAMEWORK-V18**。P0～P4、X1、X2 已完成；P5 在 2026-10-30 恢复并按完整阶段推进，**尚未通过整阶段验收**；P6 尚未验收。当前仓库先以私有方式交付进行中的成果，之后是否公开由作者另行决定。

## 已完成内容

| 部分 | 内容 | 保存成果 |
|---|---|---|
| P1 | 地图、安全缓冲、A* 与不可达 | 三场景静态图 |
| P2 | 贪心、枚举、CP-SAT、分配与路线 | 两艇四任务与反例 |
| P3 | 通信、中继、移动预算、连续服务 | 成功与受限失败案例 |
| P4 | 洋流、动态到达、故障接手、Pyomo | 同输入静态／动态策略对照 |
| X1 | 四／八邻接到执行、成本和动态融合 | 两组均完成 22/22 个任务 |
| X2 | A*、Dijkstra、D* Lite、原始 D*、扫描＋A*、PSO | 六组均完成 22/22 个任务 |

![六组算法的任务完成、实际航程与完成时间](输出文件/阶段结果/X2/X2-V01/showcase/X2_完整扩展_六组对比_V01/V02/算法效果_数据对比.png)

[六组同帧任务执行 GIF](输出文件/阶段结果/X2/X2-V01/showcase/X2_完整扩展_六组对比_V01/任务执行_同帧对比.gif) · [结果与适用范围](docs/results.md) · [复现步骤](docs/reproduction.md) · [软件与方法来源](docs/sources.md)

## 快速运行

在仓库根目录使用 Python 3.13。先创建主环境并安装依赖：

```bash
python3.13 -m venv .venv
.venv/bin/python -m pip install -r requirements-core.txt
.venv/bin/python -B tools/verify_release.py
.venv/bin/python -B -m usv_uuc_cooperation.main list
.venv/bin/python -B -m usv_uuc_cooperation.main run --stage P1 --run-id replay_p1_01
.venv/bin/python -B -m usv_uuc_cooperation.main run --stage P2 --run-id replay_p2_01
```

输出写入 `输出文件/阶段结果/阶段/版本/runs/运行名/`，摘要文件名为 `摘要.md`。运行名必须唯一，已有目录不会覆盖。P1 检查基准、增障和不可达结果；P2 分列固定起点分配成本与完整路线问题。

X2 可直接使用已保存的初始分配完成新运行，例如：

```bash
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-L1 --run-id replay_x2_astar_dijkstra_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-L2 --run-id replay_x2_dstar_lite_01
```

完整 X1／P4 求解还需要独立 Pyomo／HiGHS 环境，安装位置和命令见[复现说明](docs/reproduction.md)。MOOS 是另外构建的平台依赖，第三方源码、二进制和个人环境不包含在本仓库。

## P5 当前范围与检查点

原 P5 包含 MOOS 原例运行、分配器到 MOOS 的任务闭环、VRX/WAM-V 原例运行，以及二维模型与 VRX 的同任务对照。当前保留这套完整范围。源码已新增 MOOS 接口与日志展示。MOOS 教师工程核查已通过：分配器任务完成 1/1，原例实际运动、超时不误报完成、无效目标拒绝均有保存证据。本人运行／解释仍待完成；VRX 保留为本阶段要求，已选择 Ubuntu 24.04＋ROS 2 Jazzy＋Gazebo Harmonic 的云主机路线，租用和 SSH 入口尚待提供。整个 P5 仍为 IN_PROGRESS，P6 尚未开始。详见 [P5 平台证据](docs/p5_platform.md)。

MOOS 可用入口：

```bash
.venv/bin/python -B -m usv_uuc_cooperation.experiments.p5_moos --help
.venv/bin/python -B -m usv_uuc_cooperation.experiments.p5_moos --run-id replay_p5_moos_01 --moos-root ../moos-ivp --role learner
```

示例中的 `../moos-ivp` 是仓库旁已编译的外部 MOOS-IvP 检出，应按实际安装位置调整。`--role` 只标记运行来源；教师检查和本人运行／解释分别记录，不自动形成课程通过结论。MOOS 仿真也不替代 VRX 或实艇验证。

## 证据与边界

仓库保留运行所需冻结输入和源码 SHA 校验；少量历史目录名用于保持输入身份。源码内部旧 README 和保存数据中的历史路径属于原始来源记录，当前操作以本 README 的相对路径为准。`tools/verify_release.py` 校验本次交付文件，历史 `metrics/verify_framework.py` 依赖原课堂工作区，不作为本仓库的完整验收命令。

二维模型使用几何移动预算 NEU、距离阈值通信与理想化服务规则，未验证真实水声、电池功率或实艇性能。单一案例结果不构成所有场景的算法排名；搜索扩展、逻辑弹出和 PSO 适应度评价也不是同一工作单位。平台仿真与真实 USV／UUV 验证分别记录。

本仓库没有包含个人课堂问答、导师原文件、组会原稿、环境目录和全部历史备份。项目代码的公开许可证待作者选择，第三方软件保留各自许可；引用第三方许可不表示本项目自动采用相同许可。
