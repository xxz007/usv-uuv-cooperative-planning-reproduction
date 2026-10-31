# USV/UUV 协同规划与平台接口

当前源码版本：`FRAMEWORK-V20`，包含P5 MOOS闭环、VRX单航点控制、原生位姿记录、独立二维参考与平台运行入口。包名`usv_uuc_cooperation`沿用历史标识；正式称谓为USV/UUV。

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

VRX使用Ubuntu24.04、ROS2 Jazzy、Gazebo Harmonic及配置中固定的官方VRX提交。先由独立入口启动WAM-V及桥接，并启用原生ground-truth里程计；不得把默认传感器位姿话题当作艇体世界位姿。随后可把独立记录器与配置复制到ROS机器运行：

```bash
/usr/bin/python3 -B p5_vrx.py --config /absolute/P5-VRX-V01.json --out /absolute/new-run --role learner --framework-version FRAMEWORK-V20
```

记录器只拥有自身ROS节点，不启动或关闭Gazebo。它冻结实际观测的起点/朝向和前方20m目标，两系统从派发模拟时刻计时；观测年龄另记。VRX保存收到的原生里程计/时钟及实际推力发布，二维独立执行理想点运动学。三指标为完成时间、最大横向偏差及预定坐标包络违规事件；未完成时间保留null。到达保持依据有效收到的采样及预定最大间隔，不宣称无损传输、连续碰撞安全或真实能耗。保存数据可用`visualization.p5_vrx`生成图，不重跑平台。实际运行/验收状态以阶段摘要为准。

`experiments/p5_vrx_platform.py`可在Linux侧依序运行官方原例和完整艇体真值对照，使用独立日志、ROS域及Gazebo分区，只清理自身创建的进程组。先预取官方世界及二级模型依赖；配对URDF保留完整传感器和艇体结构，仅增加官方原生OdometryPublisher。官方原例通过Gazebo原生Pose_V按模型名称和实体ID记录世界位姿，避免TF转换丢失实体名称；时钟、推力与相机仍使用ROS。工具目录须包含控制器、配置和`VRX_官方原例记录器_V02.py`，运行参数见其`--help`。原例记录、推力发布、实际移动、相机缓冲及课程通过分别核对。

## 模块

| 目录 | 职责 |
|---|---|
| `planning/`、`env/` | 网格路径、安全几何、地图与通信代理 |
| `scheduling/` | 贪心、枚举、CP-SAT与Pyomo分配 |
| `agents/`、`control/` | 任务执行、流场、动态事件与中继 |
| `experiments/` | 各阶段运行入口，P5 MOOS接口与VRX ROS记录控制器 |
| `metrics/` | 日志、约束检查及文件身份核查 |
| `visualization/` | 由真实保存日志生成的轨迹和指标展示 |
| `configs/` | 固定场景、算法预算与平台任务配置 |

## 环境与版本

常规路径/展示环境需要networkx、numpy、matplotlib、Pillow、ortools。Pyomo和HiGHS在独立求解环境使用；X1完整重算入口当前约定求解解释器位于项目根的`课时资料/P4/P4-L4/.venv/bin/python`，创建环境与依赖版本见发布仓库说明。P5桥接使用Python标准库与MOOS现有命令行程序。

运行守卫读取`输出文件/版本快照/FRAMEWORK-V20/源码配置_SHA256.txt`。不能只复制此包而省略清单、其中列出的核查脚本及所需输入。源码实质修改后建立新版本与清单，保留旧清单；不修改旧SHA来让旧版本通过。

## 结果解释

既有P1～P4、X1、X2结果属于二维课程模型；MOOS属于平台仿真。几何NEU、距离阈值通信、理想执行与真实电池/水声/实艇验证分别记录。方法对照结论限于已执行场景；航程、系统完成时间和搜索CPU时间分别报告。未完成的任务完成时间保留null。

历史源码、原验收和恢复点保存在本地课程工作区及原版本快照中。当前发布只包含明确列出的运行依赖和代表成果；不包含私人课堂问答或导师原始资料。
