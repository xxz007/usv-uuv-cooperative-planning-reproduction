# 软件与方法来源

| 组件 | 来源与版本身份 | 本仓库处理 |
|---|---|---|
| PythonRobotics | [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)，原参考检出 `cdd0cc888802b584c2d654ca85e3c5460973487d`，MIT | 记录参考，不包含完整第三方检出 |
| MOOS-IvP | [moos-ivp/moos-ivp](https://github.com/moos-ivp/moos-ivp)，本机检出 `dd7030bdba1ea2d01524f0ff2364a3ed0d2c56a6`，GPLv3／LGPLv3，具体组件见上游 COPYING | 外部构建；平台配置和桥接单独发布 |
| Pyomo | [官方文档](https://pyomo.readthedocs.io/)，原隔离环境 6.10.1 | 声明 Python 依赖，调用真实求解接口 |
| HiGHS | [官方项目](https://highs.dev/)，原环境 highspy 1.15.1 | 声明独立求解依赖 |
| OR-Tools | [官方文档](https://developers.google.com/optimization)，原环境 9.15.6755 | CP-SAT 分配；固定缩放目标与浮点路线分列 |
| VRX | [osrf/vrx](https://github.com/osrf/vrx) | 原 P5 规划中的 WAM-V 平台；没有实际运行证据时不标为已验证 |

原始 D* 的实现按 Stentz 原算法的 h／p／k、OPEN 与回指针机制适配，保留实现说明中的差异；不声称逐字复制原始伪代码或实现 Focussed D*。扫描策略作用于任务访问顺序，单段使用 A*，不是区域全格覆盖。PSO 保留随机预算、路径表示与失败状态，不用精确搜索自动修补结果。

本项目尚未选择独立的开源许可证。上表用于说明来源，第三方依赖的许可由对应项目约束，不据此替本项目授予新的公开使用许可。
