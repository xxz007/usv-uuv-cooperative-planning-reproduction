# USV/UUV 协同规划与平台接口

当前源码版本：`FRAMEWORK-V18`，基于V15新增P5 MOOS平台适配。包名`usv_uuc_cooperation`沿用历史标识；正式称谓为USV/UUV。

此目录只保存源码、运行配置和本说明。输出写入项目根的`输出文件/`，每次运行使用新名称，既有结果不覆盖。课程通过状态由本地课程记录维护；GitHub发布说明与交付范围见发布仓库根README。

## 从项目根运行

```bash
python -B -m usv_uuc_cooperation.main list
python -B -m usv_uuc_cooperation.main run --stage P1 --run-id P1_demo_V01
python -B -m usv_uuc_cooperation.main run --stage P2 --run-id P2_demo_V01
python -B -m usv_uuc_cooperation.main compare --case X2-L1 --run-id X2_Astar_Dijkstra_demo_V01
```

完整X2可选`X2-L1`、`X2-L2`、`X2-Dstar`、`X2-Scan`、`X2-PSO`。扫描改变任务访问顺序，单段由A*连接；PSO固定有限预算搜索，不借A*修复，也不保证所有场景最优。X2运行需要已按SHA冻结的四份X1初始输入，发布包保留这些必要依赖。

P5入口独立管理本次创建的MOOS进程、工作目录与日志：

```bash
python -B -m usv_uuc_cooperation.experiments.p5_moos --help
python -B -m usv_uuc_cooperation.experiments.p5_moos --run-id P5_demo_V01 --moos-root ../moos-ivp --role learner
```

`--moos-root`指向已编译且含`bin/`的MOOS-IvP检出目录。平台工程作为外部依赖安装；仓库不打包第三方源码、构建目录或可执行程序。每次用独立运行名，输出入口为`输出文件/阶段结果/P5/P5-V01/runs/运行名/摘要.md`。运行角色仅为证据来源标记，不自动表示课程验收通过。

## 模块

| 目录 | 职责 |
|---|---|
| `planning/`、`env/` | 网格路径、安全几何、地图与通信代理 |
| `scheduling/` | 贪心、枚举、CP-SAT与Pyomo分配 |
| `agents/`、`control/` | 任务执行、流场、动态事件与中继 |
| `experiments/` | 各阶段可重复运行入口，P5真实MOOS接口 |
| `metrics/` | 日志、约束检查及文件身份核查 |
| `visualization/` | 由真实保存日志生成的轨迹和指标展示 |
| `configs/` | 固定场景、算法预算与平台任务配置 |

## 环境与版本

常规路径/展示环境需要networkx、numpy、matplotlib、Pillow、ortools。Pyomo和HiGHS在独立求解环境使用；X1完整重算入口当前约定求解解释器位于项目根的`课时资料/P4/P4-L4/.venv/bin/python`，创建环境与依赖版本见发布仓库说明。P5桥接使用Python标准库与MOOS现有命令行程序。

运行守卫读取`输出文件/版本快照/FRAMEWORK-V18/源码配置_SHA256.txt`。不能只复制此包而省略清单、其中列出的核查脚本及所需输入。源码实质修改后建立新版本与清单，保留旧清单；不修改旧SHA来让旧版本通过。

## 结果解释

既有P1～P4、X1、X2结果属于二维课程模型；MOOS属于平台仿真。几何NEU、距离阈值通信、理想执行与真实电池/水声/实艇验证分别记录。方法对照结论限于已执行场景；航程、系统完成时间和搜索CPU时间分别报告。未完成的任务完成时间保留null。

历史源码、原验收和恢复点保存在本地课程工作区及原版本快照中。当前发布只包含明确列出的运行依赖和代表成果；不包含私人课堂问答或导师原始资料。
