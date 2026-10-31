# VRX 云端环境选择｜2026-10-30

当前选定：**Ubuntu 24.04 x86_64＋ROS 2 Jazzy＋Gazebo Harmonic＋VRX jazzy 分支**。尚未租用云主机、安装远端依赖或取得实际 VRX 运行证据；P5 仍在进行。

用户先查看了 AutoDL。当前查看到的 Miniconda 和较新 PyTorch 基础镜像是 Ubuntu 22.04，社区 24.04 搜索没有匹配；这不证明平台所有镜像都不提供 24.04。用户明确选择当前官方推荐组合，因此本次不采用 Ubuntu 22.04／Humble／Garden 路线。

## 下一步候选

推荐先在阿里云 GPU 云服务器 ECS 查看单卡 T4（gn6i）或 A10（gn7i）的按量配置。阿里云官方提供 **Ubuntu 24.04 64位，预装 NVIDIA GPU 驱动与 CUDA** 的公共镜像，官方兼容列表包含 gn6i 和 gn7i。

本次建议配置：单卡 GPU、4～8 vCPU、约16～32 GiB内存、100 GiB系统盘起步，并提供可用SSH入口；优先查看8 vCPU配置的完整报价。它是待验证的租用建议，具体库存、镜像磁盘下限及费用以控制台为准。尚未下单，不按 AutoDL 截图价格推定其他平台费用。

拿到实例后先只读核对操作系统、驱动、EGL/OpenGL、磁盘与已有环境，再做最小渲染/时钟测试、官方WAM-V原例、控制输入/位姿日志及二维同任务对照。GPU和CUDA可识别不等于Gazebo渲染已通过。只有实际运行后才登记VRX证据。

## 官方依据

- [VRX系统要求](https://github.com/osrf/vrx/wiki/system_requirements)
- [VRX当前版本路线](https://github.com/osrf/vrx)
- [阿里云Ubuntu公共镜像](https://help.aliyun.com/zh/ecs/user-guide/public-mirroring-overview)
- [Ubuntu预装NVIDIA驱动镜像及兼容实例](https://help.aliyun.com/en/ecs/user-guide/ubuntu-pre-installed-nvidia-gpu-driver-image)
- [Gazebo云端EGL无界面渲染](https://gazebosim.org/api/sim/8/headless_rendering.html)
