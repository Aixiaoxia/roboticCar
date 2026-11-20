# Bunker / ROS 2 Humble

官方驱动：https://github.com/agilexrobotics/bunker_ros2 （humble 分支）。
此工作空间已加入 bunker_ros2，并使用已有 ugv_sdk 完成编译。
Scout 驱动仍保留，但请不要与 Bunker 驱动同时运行。

## 1. 构建和环境

```bash
cd ~/agx_ws
./scripts/build_humble.sh --packages-up-to bunker_base
source ~/agx_ws/setup_humble.bash
```

构建脚本使用系统 Python，避免 Conda base 环境干扰。

## 2. CAN 连接

先确认车辆已安全停放、急停和遥控接管可用。按对应车辆手册设置遥控器控制模式。
你提供的操作说明要求 B 拨到最上面；实车模式应以对应型号手册为准。

```bash
ip -details link show can0
```

接口已经 UP 且 bitrate 500000 时，不必重复配置。
若设备刚连接且接口尚未 UP，按适配器类型加载驱动并配置：

```bash
# 仅 gs_usb 类型的适配器需要这一条；当前机器识别的是 pcan_usb。
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
```

可选被动收包检查：

```bash
# 如尚未安装：sudo apt install can-utils
candump can0
```

Ctrl+C 退出收包程序。

## 3. 启动底盘（终端一）

普通 Bunker：

```bash
~/agx_ws/scripts/start_bunker.sh port_name:=can0
```

只有确认为 Bunker Mini 时使用：

```bash
~/agx_ws/scripts/start_bunker.sh port_name:=can0 is_bunker_mini:=true
```

当前上游实现中，is_bunker_mini 只用于车型提示；两种车型都通过 BunkerRobot 自动检测 V1/V2 协议。
无需改 launch 文件默认值。正常启动应显示协议识别及 Using CAN bus to talk with the robot。
若出现 Resource deadlock avoided 并退出，表示驱动崩溃，不能视为启动成功。

## 4. 查看话题（终端二）

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic list
ros2 topic info /cmd_vel --verbose
ros2 topic echo /bunker_status
```

应有 /cmd_vel（geometry_msgs/msg/Twist，底盘订阅）、/odom、/bunker_status、/bunker_rc_state。
注意：驱动自带 /odom 的 frame 默认为 odom -> base_link；接入导航时避免其他节点重复发布同一变换。
只测试底盘运动时不需要启动导航、IMU、相机或 GPS。

## 5. 人工低速测试

确认附近无人和障碍、遥控接管可用，且没有其他导航或遥控节点同时发布 /cmd_vel。
另开终端：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}'
```

linear.x 的单位为 m/s，angular.z 为 rad/s。
结束时先 Ctrl+C 停止上述速度发布，再主动连续发布零速度：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

确认车辆停止后 Ctrl+C 结束零速度发布，然后再退出底盘节点。
不要把停止发布消息等同于已发送停止指令。

## 配置验证记录

已通过 bunker_msgs、ugv_sdk、bunker_base 的编译及 rosdep 依赖检查。
配置时 can0 为 UP、500000 bit/s，被动收到了多个 CAN ID。
未启动会启用底盘控制的驱动节点，未发送运动指令，实车运动留给现场人员验证。
