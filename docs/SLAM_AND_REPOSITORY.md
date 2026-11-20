# Bunker RGB-D 建图与仓库部署

## 当前状态

| 用途 | 状态 |
| --- | --- |
| House 仿真建图 | 已验证 Bunker 模型生成、速度控制、深度图和 5 cm 栅格地图 |
| House 仿真导航 | Nav2 和两层代价地图已验证启动，尚未完成自主行驶测试 |
| 真车建图 | 本仓库包含 Bunker 和 Astra Pro 驱动，仍需实测相机外参、验证 RGB-D 数据，并补真车 RTAB-Map 启动配置 |

`agx_ws` 是项目主仓库。`rtabmap_ros` 和 RTAB-Map 核心库是独立源码依赖。仓库里的 [House 集成补丁](../patches/rtabmap_ros_bunker_house.patch) 包含 Bunker 模型、仿真启动文件、Nav2 参数覆盖和 RViz 配置。在新机器上应用补丁即可重建这些文件。

## 基础环境与 AGX 工作区

使用 Ubuntu 22.04、ROS 2 Humble Desktop、系统 Python 3.10、Git、CMake、C/C++ 编译工具、colcon 和 rosdep。进入 ROS 终端前退出 Conda。若使用 VS Code Snap 的集成终端，建议在系统终端执行仿真命令，避免其 GTK/GSettings 环境影响 RViz。

AGX 构建前先安装 Astra 驱动的原生依赖。使用系统的 `libusb-1.0-0-dev`，并按 [Astra 驱动说明](../src/ros2_astra_camera/README.MD) 安装 libuvc。本机使用 libuvc 0.0.8，`pkg-config --modversion libuvc` 应能找到它。

```bash
sudo apt-get install git build-essential cmake pkg-config python3-colcon-common-extensions \
  python3-rosdep libusb-1.0-0-dev libgflags-dev libgoogle-glog-dev libeigen3-dev libopencv-dev
git clone --recurse-submodules https://github.com/Aixiaoxia/roboticCar.git ~/agx_ws
cd ~/agx_ws
for name in apriltag_ros ros2_astra_camera ugv_sdk; do
  git -C "src/$name" apply "$PWD/patches/$name.patch" || break
done
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
./scripts/build_humble.sh
```

全新机器若还没有 rosdep 系统配置，先执行一次 `sudo rosdep init`。如果当前镜像通过 IPv6 返回 403，可用 `sudo apt-get -o Acquire::ForceIPv4=true update`，直接安装 APT 包时也可使用同一个参数。

子模块由 Git 固定提交，包括 `bunker_ros2`、`ugv_sdk`、`ros2_astra_camera` 和 `apriltag_ros`。上面的补丁只需在干净子模块上各应用一次。真车还需要 SocketCAN 兼容的 CAN 转接器、Bunker 对应的 CAN 配置、Astra Pro 的 USB 权限规则，以及 Astra 驱动所需的 libuvc/libusb。相机以 [子模块安装说明](../src/ros2_astra_camera/README.MD) 为准，底盘接线和配置见 [Bunker ROS 2](../README_BUNKER_ROS2.md)。

## 构建 House 仿真依赖

从干净的 ROS Humble 终端操作。当前 `rtabmap_ros` 源码要求 RTAB-Map **0.24.2**。本次构建使用核心库标签 `0.24.2`，提交 `8cd2f81a735722402d1e220147c4e83a1a746eaf`。Humble 镜像曾提供 0.23.14，低于这份源码的要求。

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install python3-empy libopencv-dev libpcl-dev libsqlite3-dev \
  liboctomap-dev libvtk9-qt-dev qtbase5-dev qtbase5-private-dev zlib1g-dev \
  ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-diagnostic-updater

git clone --branch 0.24.2 https://github.com/introlab/rtabmap.git ~/rtabmap
git clone https://github.com/introlab/rtabmap_ros.git ~/rtabmap_ros
git -C ~/rtabmap_ros checkout --detach f04a52d255ded6396dc11a54bfe01079ec0b0023
git -C ~/rtabmap_ros apply --check ~/agx_ws/patches/rtabmap_ros_bunker_house.patch
git -C ~/rtabmap_ros apply ~/agx_ws/patches/rtabmap_ros_bunker_house.patch

cd ~/rtabmap_ros
rosdep install --from-paths . --ignore-src --skip-keys rtabmap -y
cmake -S ~/rtabmap -B ~/rtabmap/build-humble \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HOME/.local/rtabmap-0.24.2" \
  -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build ~/rtabmap/build-humble --parallel 6
cmake --install ~/rtabmap/build-humble

export CMAKE_PREFIX_PATH="$HOME/.local/rtabmap-0.24.2${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$HOME/.local/rtabmap-0.24.2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
MAKEFLAGS=-j6 colcon build --executor sequential --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

`rosdep` 会补齐 ROS 包声明的依赖，包括 `pcl_ros`、`octomap_msgs`、`nav2_costmap_2d`，以及整个 RTAB-Map ROS 工作区中其他示例的依赖。跳过的 `rtabmap` 由上面的源码编译提供。若较旧的 `diagnostic_updater` 没有 `libdiagnostic_updater.so`，更新到包含该库的 Humble 版本。

## 启动 House

仿真使用独立 ROS 域。关闭同一域中旧的仿真实例再启动。Bunker 使用补丁自带的模型，保留 TurtleBot3 软件包中的 House 场景。

```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$HOME/.local/rtabmap-0.24.2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source ~/rtabmap_ros/install/setup.bash
export ROS_DOMAIN_ID=78
mkdir -p ~/.ros
ros2 launch rtabmap_demos bunker_house_rgbd.launch.py
```

在另外的终端加载 Humble 并使用相同域，通过 `TURTLEBOT3_MODEL=waffle ros2 run turtlebot3_teleop teleop_keyboard` 遥控仿真 Bunker。空格停车。在 RViz 中查看 `/map`、全局和局部代价地图。默认建图会重建 `~/.ros/bunker_house.db`，可通过 `database_path:=/绝对路径/新地图.db` 指定新文件。只做建图可加 `nav2:=false`。

## 真车建图还需接线和配置

1. 真车使用 Bunker 驱动的 `/odom` 与 `odom → base_link`。选择与仿真分开的 ROS 域。
2. 实测相机位姿并配置 `base_link → camera_link`，保留 Astra 驱动发布的内部光学 TF。
3. 验证 RGB、已配准深度和相机内参话题的尺寸、时间戳与坐标系。Astra Pro 的硬件深度配准默认关闭，须根据实际驱动输出选择对齐方法。
4. 加载已构建的 RTAB-Map 与 AGX 工作区，增加启动配置并映射上述话题，使用 `frame_id:=base_link` 和 `use_sim_time:=false`。先检查静止时的 TF 和 RGB-D 数据，再人工低速移动验证地图。真车首次建图无需 Nav2 和 AprilTag 跟随。

## 提交和推送

当前 `agx_ws` 已有 Git 仓库，`origin` 指向 `Aixiaoxia/roboticCar`。先用 `git remote -v` 确认目的地。提交源代码、说明、`.gitmodules` 和补丁。构建目录、安装目录、日志、实测地图数据库和私有采集数据留在本地。子模块内的本地修改不会随主仓库推送，当前三份已有补丁用于复现这些修改。

```bash
cd ~/agx_ws
git status --short
git diff --submodule
git add .gitignore README_HUMBLE.md docs/SLAM_AND_REPOSITORY.md patches/rtabmap_ros_bunker_house.patch
git diff --cached --check
git diff --cached --stat
git commit -m "Document Bunker SLAM dependencies and package House simulation"
git push origin main
```
