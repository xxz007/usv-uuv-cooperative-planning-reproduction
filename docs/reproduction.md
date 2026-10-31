# 复现步骤与验证范围

## 两个 Python 环境

主环境用于路径、OR-Tools 分配、二维执行和绘图：

```bash
python3.13 -m venv .venv
.venv/bin/python -m pip install -r requirements-core.txt
```

Pyomo／HiGHS 使用独立环境；X1 完整融合入口目前按以下仓库相对路径寻找解释器：

```bash
python3.13 -m venv 课时资料/P4/P4-L4/.venv
课时资料/P4/P4-L4/.venv/bin/python -m pip install -r requirements-solver.txt
.venv/bin/python -B -m usv_uuc_cooperation.main integrate --case X1-L4 --run-id replay_x1_full_01
```

P4 可先求解初始分配，再由主环境执行两个响应策略：

```bash
课时资料/P4/P4-L4/.venv/bin/python -B -m usv_uuc_cooperation.main solve --stage P4 --run-id replay_p4_initial_01
.venv/bin/python -B -m usv_uuc_cooperation.main run --stage P4 --case ALL --run-id replay_p4_pair_01 --initial-result 输出文件/阶段结果/P4/P4-V01/runs/replay_p4_initial_01/初始分配.json
```

两个 requirements 文件记录直接依赖版本。P6已从远端固定提交新克隆，在现有macOS主机用新隔离venv重新安装主环境依赖，真实执行A*／Dijkstra核心对比并制图，确定性数据与原基线一致；19项解析依赖另存requirements-reproduced-macos.txt。独立求解环境本轮没有重装，跨系统与完整平台性能需要另行核对。[实际P6复跑与原始记录](p6_delivery.md)

## 可选的新运行

```bash
.venv/bin/python -B -m usv_uuc_cooperation.main run --stage P3 --case ALL --run-id replay_p3_pair_01
.venv/bin/python -B -m usv_uuc_cooperation.main plan --case X1-L1 --run-id replay_x1_path_01
.venv/bin/python -B -m usv_uuc_cooperation.main execute --case X1-L2 --run-id replay_x1_execute_01
.venv/bin/python -B -m usv_uuc_cooperation.main assign --case X1-L3 --phase reassign --run-id replay_x1_assign_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-Dstar --run-id replay_x2_original_dstar_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-Scan --run-id replay_x2_scan_01
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-PSO --run-id replay_x2_pso_01
```

以上是当前命令入口，不表示发布准备时又执行了全部完整实验。已保存的展示图沿用原运行，重新运行输出使用新目录。每次先看运行目录的 `摘要.md` 与核查 JSON，再按需要查看原始日志。

## P5：平台新执行与保存日志复核

P5 已完整通过，当前交付 FRAMEWORK-V20；本人原始运行与教师工程证据分别保存。MOOS 需要外部编译的 MOOS-IvP，以下路径应调整为实际安装位置：

```bash
.venv/bin/python -B -m usv_uuc_cooperation.experiments.p5_moos --run-id replay_p5_moos_01 --moos-root ../moos-ivp --role learner
```

Linux VRX 使用系统 Python 3.12、ROS 2 Jazzy、Gazebo Harmonic 和固定上游 pin；按 [VRX 环境说明](vrx_environment.md)预取 13 个模型、生成完整传感器 GT URDF，并准备四个工具文件后，用统一入口 `--mode all --role learner` 执行。原例与同任务对照分别启动／关停、隔离域和日志，不拼接轨迹；未就绪不发任务推力，失败保留原始输出。

只核查发布的本人保存数据，无需 MOOS／ROS／Gazebo，可使用 [P5 平台页的三条保存核查命令](p5_platform.md#只重查保存日志)。其原始 CSV、RGB、启动元数据和独立核查均已发布。重新绘制配对图同样只读保存数据，使用新的输出目录：

```bash
.venv/bin/python -B -m usv_uuc_cooperation.visualization.p5_vrx --source-run "$PWD/输出文件/阶段结果/P5/P5-V01/vrx_runs/P5_本人VRX_V01/pair/observations" --output "$PWD/输出文件/阶段结果/P5/P5-V01/showcase/replay_p5_saved_plot_01"
```

实际运行数值受平台、离散采样和执行时序影响。冻结代码、输入和资产身份可核对，不等于保证不同机器逐数值／逐字节一致。`--role` 只是运行来源；脚本不会自动判定本人理解或整阶段课程通过。[已完成阶段验收](p5_acceptance.json)

## 校验与搬迁

```bash
.venv/bin/python -B tools/verify_release.py
.venv/bin/python -B -c 'from usv_uuc_cooperation.main import assert_frozen_core; assert_frozen_core(); print("source guard PASS")'
```

当前 FRAMEWORK-V20 含 106 项源码／配置和 4 项输入依赖。`RELEASE_CONTENTS.json` 记录本次交付文件大小和 SHA256；`源码配置_SHA256.txt` 约束当前源码、配置和少量核查脚本。修改生产代码后应创建新版本，保留旧清单。源文件校验通过只说明身份一致，不等同于所有算法或平台正确性证明。

本仓库保留 X2 依赖的四份 X1 输入、X1-L2 依赖的路径结果和 X1-L3 的小案例配置，不要求获取完整个人课程工作区。历史课堂审计器需要更多原始记录，本次发布没有提供其全套输入。P5 完整阶段恢复包已在原工作区完成独立恢复及保存日志核查，但包含私人课程资料，不随本仓库发布；仓库范围使用上述发布清单与保存核查入口。
