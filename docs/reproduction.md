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

两个 requirements 文件记录原工作区观察到的直接依赖版本。发布准备阶段复用了已有环境做搬迁冒烟，未在全新系统重新安装这两个环境；不同系统上的安装和完整性能复现需要单独核对。

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

## 校验与搬迁

```bash
.venv/bin/python -B tools/verify_release.py
.venv/bin/python -B -c 'from usv_uuc_cooperation.main import assert_frozen_core; assert_frozen_core(); print("source guard PASS")'
```

`RELEASE_CONTENTS.json` 记录本次交付文件大小和 SHA256；`源码配置_SHA256.txt` 约束当前源码、配置和少量核查脚本。修改生产代码后应创建新版本，保留旧清单。源文件校验通过只说明身份一致，不等同于所有算法或平台正确性证明。

本仓库保留 X2 依赖的四份 X1 输入、X1-L2 依赖的路径结果和 X1-L3 的小案例配置，不要求获取完整个人课程工作区。历史课堂审计器需要更多原始记录，本次发布没有提供其全套输入。
