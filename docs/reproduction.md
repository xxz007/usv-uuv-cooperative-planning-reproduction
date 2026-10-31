# 复现说明

先在仓库根运行所有命令。Python主环境与Pyomo求解环境分别安装；MOOS／VRX为外部平台，不随仓库提供。下列运行名应按实际尝试更换。

## 主环境与已保存结果

```bash
python3.13 -m venv .venv
.venv/bin/python -m pip install -r requirements/core.txt
.venv/bin/python -B tools/verify_release.py
.venv/bin/python -B tools/verify_saved_results.py
.venv/bin/python -B -m usv_uuc_cooperation.main list
```

前两项核查分别检查当前文件身份与已保存X2／P3数据，不启动模拟。`requirements/macos-lock.txt`保存P6在macOS／Python3.13.15解析的依赖；主环境直接依赖见core.txt。其他系统的wheel与字体需另行核对。

## 从小案例到完整动态对比

| 主题 | 命令中`main`之后的参数 | 输出目录 |
|---|---|---|
| P1三场景 | `run --stage P1 --run-id learn_p1_01` | `results/p1_pathfinding/runs/learn_p1_01/` |
| P2分配 | `run --stage P2 --run-id learn_p2_01` | `results/p2_assignment/runs/learn_p2_01/` |
| P3成功／受限 | `run --stage P3 --case ALL --run-id learn_p3_01` | `results/p3_baseline/runs/learn_p3_01/` |
| X1四八邻接 | `plan --case X1-L1 --run-id learn_paths_01` | `results/x1_adjacency/runs/learn_paths_01/` |
| X1原路径执行 | `execute --case X1-L2 --run-id learn_execute_01` | `results/x1_adjacency/runs/learn_execute_01/` |
| X1重新分配 | `assign --case X1-L3 --phase reassign --run-id learn_assign_01` | `results/x1_adjacency/runs/learn_assign_01/` |

完整命令前缀为`.venv/bin/python -B -m usv_uuc_cooperation.main`。先打开输出中的`摘要.md`，再按需要检查原始数据；每次只关注完成数、实际航程、团队完成时间等当前问题所需读数。

## X2六种路径方法

```bash
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-L1 --run-id learn_astar_dijkstra_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-L2 --run-id learn_dstar_lite_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-Dstar --run-id learn_dstar_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-Scan --run-id learn_scan_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-PSO --run-id learn_pso_01
```

输出统一在`results/x2_algorithms/runs/<运行名>/`。A*／Dijkstra的已保存精选数据可独立复核；六算法已有对比表与图。当前精选包没有包含其余四种方法的全部历史原始日志，完整重绘六组动画需作者本地课堂完整日志，或先产生六组新运行；[来源说明](sources.md)中区分了已发布旧材料和本地原件。

仅从新A*／Dijkstra保存数据制图：

```bash
.venv/bin/python -B -m usv_uuc_cooperation.main render --stage X2 --run "$PWD/results/x2_algorithms/runs/learn_astar_dijkstra_01" --revision V01
```

输出为`results/x2_algorithms/showcase/learn_astar_dijkstra_01/V01/`，不重新模拟。其他单算法运行也使用对应的X2制图入口；已有图不覆盖。

## 独立Pyomo／HiGHS环境

```bash
python3.13 -m venv .venv-solver
.venv-solver/bin/python -m pip install -r requirements/solver.txt
.venv-solver/bin/python -B -m usv_uuc_cooperation.main solve --stage P4 --run-id learn_initial_01
.venv/bin/python -B -m usv_uuc_cooperation.main run --stage P4 --case ALL --run-id learn_dynamic_01 --initial-result "$PWD/results/p4_dynamic/runs/learn_initial_01/初始分配.json"
.venv/bin/python -B -m usv_uuc_cooperation.main integrate --case X1-L4 --run-id learn_x1_full_01
```

P4比较固定归属和动态响应；X1完整比较分别在四／八邻接下求初始分配，再执行相同动态策略。X2复用六份固定输入中的既有来源，不依赖启动求解器。

## 路径与版本

六份历史输入迁入`data/inputs/`，原始字节／SHA保持，源码仅更新真实读取路径。预览FRAMEWORK-V21保存当前源码／输入身份，V20仍由原课程与[基线提交](sources.md)保留。框架校验通过表示身份一致，不能代替算法正确性与物理验证。

P6曾验证真实远端副本＋新venv的A*／Dijkstra；本次目录预览复用已安装依赖，另做输入加载、核心完整运行、制图与保存日志核查。实际验证结果见[迁移与整合验证](../results/p6_reproduction/README.md)，不把工程运行登记为本人课堂操作。

MOOS／VRX的外部依赖、保存日志复核和平台启动见[平台说明](platforms.md)。
