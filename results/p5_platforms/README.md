# MOOS与VRX单艇结果

MOOS任务1/1、19.08模拟秒，理想二维13.75秒；VRX同任务15.7秒，理想二维12.7秒。两平台及原例各自独立运行。

![MOOS航迹](figures/MOOS_learner_same_task.png)

![VRX空间路线](figures/2D_VRX_same_task.png)

![VRX进度与误差](figures/VRX_progress_error.png)

## 第三人称外部视角

独立实际场景取景，未发布推进指令。相机用于观察完整WAM-V与环境，没有接入规划和控制；这些画面与任务性能运行分别记录。

![侧面近景](figures/VRX_external_detail.png)

![艇首斜侧方](figures/VRX_external_bowquarter.png)

![环境景](figures/VRX_external_shoreline.png)

外部图均1600×900原生RGB，未裁切或补画；[相机位姿与图像来源](camera_views.json)。

## 艇载相机

原例实收1280×720 RGB8首帧；3秒零推力后左右各100 N运行8秒，采样位移6.093563239 m。

![艇载首帧](figures/original_first_camera_frame.png)

[最终指标](metrics.json)

[项目结果总览](../../README.md#5复现结果) · [运行指南](../../README.md#6运行指南)
