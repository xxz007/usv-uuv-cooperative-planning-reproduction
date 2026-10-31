# USV／UUV 协同任务规划：复现与学习记录

从一个可解释的二维案例出发，学习路径搜索、任务分配和动态执行，再把选定接口接入MOOS与VRX。基础场景为 **1艘USV＋5艘UUV、20个初始任务**；动态实验增加两个任务，并引入洋流、临时禁行和艇故障。

本仓库提供运行代码、整理后的中文学习笔记、代表结果及复核入口。课程主线P0～P6、X1、X2已完成；导师提出的六个科研方向仍有后续研究空间。[完成范围与局限](docs/scope.md)

> 当前是结构整理预览：`preview/learning-layout-v1`。原交付版本保留在main与历史提交。预览代码标记FRAMEWORK-V21，修改集中在目录、输入和入口路径，规划与控制算法沿用V20。

![六种方法在同一动态案例中的结果](results/x2_algorithms/figures/comparison.png)

## 从这里开始

| 想做什么 | 入口 |
|---|---|
| 按顺序学习 | [学习路线与六篇主题笔记](docs/learning/README.md) |
| 安装并运行 | [复现说明](docs/reproduction.md) |
| 查看核心结果 | [结果对比与统计口径](docs/results.md) |
| 了解平台实验 | [MOOS／VRX](docs/platforms.md) |
| 查找来源与完整旧材料 | [资料来源与归档](docs/sources.md) |

## 快速运行一个完整对比

在仓库根目录使用Python 3.13，先建立主环境：

```bash
python3.13 -m venv .venv
.venv/bin/python -m pip install -r requirements/core.txt
.venv/bin/python -B tools/verify_release.py
.venv/bin/python -B -m usv_uuc_cooperation.main compare --case X2-L1 --run-id first_comparison
.venv/bin/python -B -m usv_uuc_cooperation.main render --stage X2 --run "$PWD/results/x2_algorithms/runs/first_comparison" --revision V01
```

运行摘要：`results/x2_algorithms/runs/first_comparison/摘要.md`。图位于对应的`showcase/first_comparison/V01/`。运行名必须唯一，已有结果不会覆盖。预期三项读数：两组均 **22/22任务、122.598182196米、54秒**。

## 学习与验证的内容

| 主题 | 主要内容 | 代表成果 |
|---|---|---|
| 路径与分配 | 安全栅格、A*、贪心／枚举／CP-SAT／Pyomo | [路径案例](results/p1_pathfinding/README.md)、[分配反例](results/p2_assignment/README.md) |
| 协同执行 | 任务顺序、中继、服务、预算、洋流与故障接手 | [成功／受限失败](results/p3_baseline/README.md)、[静态／动态](results/p4_dynamic/README.md) |
| 算法比较 | 四／八邻接及六种路径方法 | [X1](results/x1_adjacency/README.md)、[X2](results/x2_algorithms/README.md) |
| 平台接口 | 单艇MOOS闭环、单WAM-V原例与二维对照 | [平台结果](results/p5_platforms/README.md) |
| 可复现交付 | 新副本复跑、来源校验与保存数据独立核查 | [整合与迁移验证](results/p6_reproduction/README.md) |

![同一二维基线中的任务执行](results/p3_baseline/figures/success.gif)

## 目录

```text
usv_uuc_cooperation/   可运行源码与配置
docs/                 学习笔记、复现方法、结果解释和来源
data/inputs/          被程序实际使用的固定输入
data/provenance/      当前版本与来源校验
results/              代表图、指标和必要原始数据
tools/                内容与保存结果核查
requirements/         主环境、求解环境及已记录依赖
```

新运行生成的`runs/`、`showcase/`和`checks/`默认不提交。历次课堂问答、备份、完整日志和多版验收报告通过原版本归档查阅，当前目录保留学习和复现所需内容。

二维通信采用距离阈值，能耗采用几何NEU代理。MOOS与VRX分别为单艇实验；本项目尚未验证真实水声、物理电池功耗、水下六自由度、多艇平台联动或实艇性能。单场景比较不构成普遍算法排名。
