# 来源、版本与完整材料

项目依据作者收到的导师研究路线整理，采用USV／UUV正式称谓；代码包及少量原始字段中的UUC为历史名称。本文记录方法和数据来源，不把其他人的实验室记录当作本项目执行证据。

## 软件与方法

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)：路径规划学习来源，具体实现边界见各主题笔记。
- [OR-Tools](https://developers.google.com/optimization/)与[Pyomo](https://www.pyomo.org/)：分配建模及求解；[NetworkX](https://networkx.org/)：几何通信图。
- [MOOS-IvP](https://oceanai.mit.edu/moos-ivp/)：独立构建的平台；[VRX](https://github.com/osrf/vrx)：WAM-V平台，实际pin和模型身份见[环境说明](vrx_environment.md)。

第三方软件保留各自许可。本项目代码的公开许可证待作者选择；第三方许可不自动决定本项目许可。

## 当前精选数据

[来源索引](../data/provenance/sources.json)记录输入／代表图／指标的源路径与SHA。六份运行输入原字节保留；P3成功／失败、X2两组、P5本人主运行保留足以核查或重绘的数据。P4静态图是原GIF的t65帧，未改轨迹或任务状态。

[版本与源码输入校验](../data/provenance/FRAMEWORK-V21/version.json) · [当前文件清单](../data/provenance/release.json)

FRAMEWORK-V21是目录预览版；原课程FRAMEWORK-V20和验收原件保留。新运行输出与保存参考分开，工程验证不增加学员执行或理解通过记录。

本次新增的 [P5第三人称实景](../results/p5_platforms/README.md#第三人称外部视角) 来自独立VRX实际取景，原生RGB像素与PNG逐SHA核对。侧面／环境景与追加艇首斜侧方各有独立取景记录；观察相机、具体位姿和模拟时刻单独记录，未发布推进指令，不混记为旧本人任务或新算法验证。

## 已发布旧材料与本地原件

[P6交付基线：6b2c7b2](https://github.com/xxz007/usv-uuv-cooperative-planning-reproduction/tree/6b2c7b2476f63fb07875afecf2bdc6bd6fadf934)保留整理前的全部已发布材料，包括较长动画、完整发布记录及多版恢复证明。原课程的私人课堂回答、本地完整恢复包及其余四种算法的完整历史日志没有加入GitHub；当前GitHub归档包含已发布的指标／图／动画，重绘全算法动画须取得本地完整日志或先产生六组新运行。

- [当前直接展示的六算法同帧动画](../results/x2_algorithms/figures/six_algorithms_execution.gif)：与交付基线字节一致。
- [原导师要求逐项评估](https://github.com/xxz007/usv-uuv-cooperative-planning-reproduction/blob/6b2c7b2476f63fb07875afecf2bdc6bd6fadf934/docs/project_assessment.md)
- [一页候选研究问题卡](https://github.com/xxz007/usv-uuv-cooperative-planning-reproduction/blob/6b2c7b2476f63fb07875afecf2bdc6bd6fadf934/docs/research_question.md)

本次通过独立预览分支试做，main和完整历史保持。以后采用精简布局时，可将完整材料另作版本附件；当前不重写Git历史。
