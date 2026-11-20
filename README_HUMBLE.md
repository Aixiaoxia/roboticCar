# ROS 2 Humble 工作空间

此工作空间包含 ugv_sdk 和 scout_ros2（humble 分支），适用于 Scout 系列。
ugv_sdk/package.xml 已将 catkin 构建声明改为 cmake，使用 SDK 现有的独立 CMake 安装导出。

构建（脚本使用系统 Python，避免 Conda 干扰）：

```bash
cd ~/agx_ws
./scripts/build_humble.sh
```

每个新终端加载环境：

```bash
source ~/agx_ws/setup_humble.bash
```

依赖检查：

```bash
rosdep check --from-paths ~/agx_ws/src --ignore-src
```

实车接入：先连接 SocketCAN 兼容的 CAN 设备，确认实际接口名称。
以下假设接口为 can0、底盘 CAN 波特率为 500000。

```bash
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
ros2 launch scout_base scout_base.launch.py port_name:=can0
```

Scout Mini 使用 is_scout_mini:=true；Mini 全向轮还需 is_omni_wheel:=true。
启动前按对应底盘手册切到允许程序控制的模式，确认急停和周围空间。
驱动订阅 /cmd_vel（geometry_msgs/msg/Twist），linear.x 为 m/s，angular.z 为 rad/s。
首次测试使用低速，确保可以用遥控器或急停接管。
当前配置未验证实车通信，未发送任何运动指令。

## 实际车型更新：Bunker

此工作空间已添加 Bunker ROS 2 驱动。实际车辆是 Bunker，请使用
README_BUNKER_ROS2.md 中的命令，不要启动上文 Scout 驱动。

## AprilTag 跟随预览

新增 src/bunker_tag_follower，手持相机先运行 `~/agx_ws/scripts/start_tag_follow.sh`。
该默认模式只发布预览速度；完整结构、测试和后续实车条件见 src/bunker_tag_follower/README.md。

完整操作文档：[Bunker + Astra + AprilTag 使用指南](docs/Bunker_AprilTag_ROS2_完整使用指南.md)。

## 一键总启动

退出旧的单独启动节点后，使用 `~/agx_ws/scripts/start_all.sh`。
该入口统一启动相机、识别、可视化、底盘、跟随、深度监控及控制窗口；启动后保持禁用。
使能和停止可在窗口中点击。详细参数见完整使用指南的“一键启动”部分。

## 新克隆的工作空间

在仓库根目录初始化依赖并应用本项目的本地改动，然后运行上文的构建命令。

```bash
git submodule update --init --recursive
for name in apriltag_ros ros2_astra_camera ugv_sdk; do
    git -C "src/$name" apply "$PWD/patches/$name.patch" || exit 1
done
```
