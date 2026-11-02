# VRX 实际环境与 Linux 复现

使用兼容的Ubuntu 24.04 x86_64主机与系统Python 3.12。ROS／Gazebo／VRX、Fuel模型和GPU环境由使用者另行准备。以下保留固定版本、必要环境步骤与运行命令。

## 参考运行环境

| 组件 | 实际身份 |
|---|---|
| 系统／ROS Python | Ubuntu 24.04.1、系统 Python 3.12.3、ROS 2 Jazzy |
| Gazebo | Harmonic；gz-sim 8.15.0、fuel-tools 9.1.1 |
| 原生 Python | transport13 13.6.0、msgs10 10.4.0、sdformat14 14.9.0 |
| ROS 桥 | ros_gz 1.0.24、ros-base 0.11.0 |
| GPU／渲染 | RTX 2080 Ti 11GB、NVIDIA 595.71.05、OGRE2 实测 OpenGL 4.5 |
| VRX pin | `7609d1bd90ce7edb29d040a082f949e8b089c864` |


这些是原运行身份，不保证所有机器数值或文件字节相同。CUDA 可见不等于 EGL／相机渲染可用。原例使用原生 Pose_V 的 name／id，ROS 用于时钟、推力和图像；对照使用原生真值 Odometry。

## 官方环境与固定源码

先按 [ROS Jazzy Ubuntu deb](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)和 [Gazebo Harmonic 安装](https://gazebosim.org/docs/harmonic/install_ubuntu/)配置官方 apt 源；主机应提供可用 NVIDIA 驱动／EGL，见 [OGRE2 无头渲染](https://gazebosim.org/api/sim/8/headless_rendering.html)。再安装：

```bash
sudo apt-get update
sudo apt-get install --no-install-recommends \
  build-essential cmake git curl unzip libeigen3-dev procps \
  python3-colcon-common-extensions python3-sdformat14 \
  python3-gz-transport13 python3-gz-msgs10 \
  ros-jazzy-ros-base ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-image ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-ros-gz-interfaces gz-harmonic
```

从本仓库根设置路径，`P5_ROOT` 选择新的外部目录，避免把第三方代码／大型模型加入仓库：

```bash
P5_REPO="$PWD"
P5_ROOT="$HOME/p5-vrx-repro"
P5_WS="$P5_ROOT/vrx_ws"
P5_CACHE="$P5_ROOT/fuel-cache"
P5_TOOLS="$P5_ROOT/tools"
P5_URDF="$P5_ROOT/derived/wamv_full_gt_V01.urdf"
mkdir -p "$P5_WS/src" "$P5_CACHE" "$P5_TOOLS" "$P5_ROOT/derived" "$P5_ROOT/downloads"
git clone --branch jazzy https://github.com/osrf/vrx.git "$P5_WS/src/vrx"
git -C "$P5_WS/src/vrx" checkout --detach 7609d1bd90ce7edb29d040a082f949e8b089c864
source /opt/ros/jazzy/setup.bash
cd "$P5_WS"
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$P5_WS/install/setup.bash"
export GZ_FUEL_CACHE_PATH="$P5_CACHE"
/usr/bin/python3 -c 'import sdformat14; from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V; print("native imports PASS")'
```

沿用[官方 VRX 安装](https://github.com/osrf/vrx/wiki/installation_tutorial)，不用 `--symlink-install`。若导入失败，先解决依赖／库版本。apt 获取当时可用包，版本不同应记录为新环境；不要用 Python 3.13 虚拟环境替换 ROS 的系统 Python。

## 13 个 Fuel 模型及实际版本

缓存共13模型、117文件、约162 MB；[资产清单](../data/provenance/vrx/fuel_cache_identity.json)保存文件SHA／字节数。所有者均为 openrobotics。Git pin 不冻结 Fuel 资产；目录版本是本次实际解析值，Sydney3／chair3 另有官方响应版本和 archive SHA。

| 模型 | 版本 | 模型 | 版本 |
|---|---:|---|---:|
| antenna | 5 | ground_station | 7 |
| mb_marker_buoy_black | 7 | mb_marker_buoy_green | 4 |
| mb_marker_buoy_red | 5 | mb_marker_buoy_white | 4 |
| mb_round_buoy_black | 4 | mb_round_buoy_orange | 5 |
| post | 4 | sydney_regatta | 3 |
| tent | 4 | foldable_table | 4 |
| foldable_chair | 3 | | |

在新缓存中先用原生客户端下载 11 项。`-v 4` 是日志级别；资源版本在 URL 末尾，不用动态 tip：

```bash
while read -r P5_MODEL P5_VERSION; do
  gz fuel download -v 4 -u "https://fuel.gazebosim.org/1.0/openrobotics/models/$P5_MODEL/$P5_VERSION" || break
done <<'MODELS'
antenna 5
ground_station 7
mb_marker_buoy_black 7
mb_marker_buoy_green 4
mb_marker_buoy_red 5
mb_marker_buoy_white 4
mb_round_buoy_black 4
mb_round_buoy_orange 5
post 4
tent 4
foldable_table 4
MODELS
```

本次 Sydney3／chair3 从官方完整 ZIP 导入，没有改 ZIP／SDF；SDF、DAE 引用均已检查在包内。以下保留响应头、核对原 archive SHA，已有模型目录不覆盖：

```bash
for P5_MODEL in sydney_regatta foldable_chair; do
  P5_ARCHIVE="$P5_ROOT/downloads/${P5_MODEL}_v3.zip"
  curl --fail --location --retry 3 \
    --dump-header "$P5_ROOT/downloads/${P5_MODEL}_v3.headers" \
    --output "$P5_ARCHIVE" \
    "https://fuel.gazebosim.org/1.0/openrobotics/models/$P5_MODEL/3/$P5_MODEL.zip" || break
  if [ "$P5_MODEL" = sydney_regatta ]; then
    P5_EXPECTED=ccf768134ff7e7929f67970fd059c0286569bc22ead9d959d45061a5c1ee967a
  else
    P5_EXPECTED=2b0ea77947f568db468f55e39c91647ce5c8091016daed5d2bcea2737158dc82
  fi
  printf '%s  %s\n' "$P5_EXPECTED" "$P5_ARCHIVE" | sha256sum --check - || break
  unzip -t "$P5_ARCHIVE" || break
  P5_MODEL_DIR="$P5_CACHE/fuel.gazebosim.org/openrobotics/models/$P5_MODEL/3"
  test ! -e "$P5_MODEL_DIR" || break
  mkdir -p "$P5_MODEL_DIR"
  unzip -q "$P5_ARCHIVE" -d "$P5_MODEL_DIR" || break
done
```

确认官方 API 响应头 `X-Ign-Resource-Version` 为 3；SHA 或版本不符时停止，保留差异记录。Fuel 9 缺失版本头可能回退目录 1，且原生导入会处理 SDF URI／格式，因此不能仅凭目录名宣称来源／字节一致。[Fuel CLI](https://gazebosim.org/api/fuel_tools/9/cmdline.html) · [缓存配置](https://gazebosim.org/api/fuel_tools/9/configuration.html) · [下载实现](https://github.com/gazebosim/gz-fuel-tools/blob/gz-fuel-tools9/src/FuelClient.cc)

核对保存的 117 文件身份；不一致应另存新环境记录，不改旧清单。所有直接及二级资源齐全后才运行：

```bash
/usr/bin/python3 - "$P5_REPO/data/provenance/vrx/fuel_cache_identity.json" "$P5_CACHE" <<'PY'
import hashlib, json, sys
from pathlib import Path
expected, cache = json.loads(Path(sys.argv[1]).read_text()), Path(sys.argv[2])
different = []
for name, identity in expected['files'].items():
    path = cache / name
    if not path.is_file() or path.stat().st_size != identity['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest() != identity['sha256']:
        different.append(name)
print(json.dumps({'checked_files': len(expected['files']), 'different': different}, indent=2))
raise SystemExit(2 if different else 0)
PY
```

## 完整 GT 船体与四文件工具

对照使用完整传感器船体，只增加 OdometryPublisher，保留质量／惯量、H 推进、浮力和阻尼；两个开关都是[顶层 xacro 参数](https://github.com/osrf/vrx/blob/7609d1bd90ce7edb29d040a082f949e8b089c864/vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro)，无需 wrapper：

```bash
xacro -o "$P5_URDF" \
  "$(ros2 pkg prefix --share wamv_gazebo)/urdf/wamv_gazebo.urdf.xacro" \
  namespace:=wamv locked:=true thruster_config:=H \
  vrx_sensors_enabled:=true ground_truth_enabled:=true
cp "$P5_REPO/usv_uuc_cooperation/experiments/p5_vrx.py" "$P5_TOOLS/"
cp "$P5_REPO/usv_uuc_cooperation/experiments/p5_vrx_platform.py" "$P5_TOOLS/"
cp "$P5_REPO/usv_uuc_cooperation/configs/P5-VRX-V01.json" "$P5_TOOLS/"
cp "$P5_REPO/tools/platforms/VRX_官方原例记录器_V02.py" "$P5_TOOLS/"
```

使用新的 URDF 文件，完全展开后再传给启动器。关闭 VRX 传感器组合会同时移除 CPU 盒等质量，不是本次对照配置。原例不传派生 URDF，仍使用官方默认船体；对照传 GT 文件。[上游模型生成链](https://github.com/osrf/vrx/blob/7609d1bd90ce7edb29d040a082f949e8b089c864/vrx_gz/src/vrx_gz/model.py)

## 一条命令运行

在上述变量有效的同一 shell 中，用新运行名执行：

```bash
/usr/bin/python3 -B "$P5_TOOLS/p5_vrx_platform.py" \
  --mode all --role learner --run-id replay_p5_all_01 \
  --out-root "$P5_ROOT/runs" --vrx-workspace "$P5_WS" \
  --fuel-cache "$P5_CACHE" --tools-dir "$P5_TOOLS" \
  --urdf "$P5_URDF" --framework-version FRAMEWORK-V22
```

先看 `$P5_ROOT/runs/replay_p5_all_01/summary.json`，再看各自 `original/observations/摘要.md`、`pair/observations/摘要.md`。可单选 `--mode original` 或 `pair`。子运行独立 ROS 域／Gazebo partition，保留 100 墙秒真实反馈就绪窗；原例还须实收 Image。未就绪不启动任务，原例失败则 all 停止；只回收本次拥有的进程组。保存实际 Git pin／修改、工具／URDF SHA 和清理记录，上游 pin 不符或有受跟踪修改会阻止运行。

原例为 3 模拟秒零推力＋8 秒左右各 100 N；对照冻结实测起点前方 20 m、标称 1.5 m/s、半径 2.5 m、保持 1 s、预算 90 模拟秒。角色仅为运行来源标签，不影响实验参数。[平台成果与指标](../results/p5_platforms/README.md)
