# 参数配置入口

统一编辑 `~/agx_ws/follow_config.yaml`。限速只写一处，两节点共用；文件修改后重启生效。
详见 [跟随参数配置说明](../../docs/跟随参数配置说明.md)。

# 新增：AprilTag 平面 KF

普通 KF 默认启用，首帧无额外等待，目标丢失后仍停车并重置。
实现见 `bunker_tag_follower/kalman.py`；详细参数和验证步骤见 [KF 使用说明](../../docs/AprilTag_KF平面滤波使用说明.md)。

# 一键启动（推荐）

先禁用旧跟随并退出旧相机、检测、叠加、底盘和跟随启动终端，然后执行：

```bash
~/agx_ws/scripts/start_all.sh
```

一个终端启动全部节点及图像/状态/使能停止控制窗口。默认保持禁用，点击窗口“使能跟随”才可能运动。
完整操作步骤和可选参数见 [使用指南的一键启动部分](../../docs/Bunker_AprilTag_ROS2_完整使用指南.md)。
ROS 入口为 `ros2 launch bunker_tag_follower bringup.launch.py`；无桌面用 `panel:=false`。

# 当前更新：固定相机实车通道

完整现场步骤见 [完整使用指南第 0 节](../../docs/Bunker_AprilTag_ROS2_完整使用指南.md)。

```bash
~/agx_ws/scripts/start_tag_follow.sh mode:=mounted hardware:=true centered_forward_mount:=true start_depth_monitor:=true
```

此启动保持禁用，人工检查目标、深度和底盘状态后才使能。`centered_forward_mount` 仅适用于居中且水平朝前的固定相机，不发布三维外参。
目标距离已改为 0.5 m；近障停车阈值独立为 0.6 m，必要时会先于目标距离停车；不能保证车头留有 0.5 m 净空。
新增 `depth_node.py`、`depth_policy.py`；`/follow/depth_state` 给出有效率、最近距离和拦截原因。

# Bunker AprilTag 跟随（ROS 2 Humble）

完整操作步骤见 [Bunker AprilTag ROS 2 完整使用指南](../../docs/Bunker_AprilTag_ROS2_完整使用指南.md)。

针对本工作空间的 Bunker、Astra Pro、tag36h11 ID 0（黑框边长 0.15 m）。
参考 `~/Robot-QR-code` 的“目标位姿 -> 距离/偏角 -> 速度”思路重新实现，未复制其中的代码。
原项目依赖 ROS 1 nodelet/visp/Powerbot；其回调中循环使用旧距离、混合距离误差与转向、缺少独立失联保护，不适合直接替换 API 后使用。
本包不依赖 ViSP，也不重新跑一次图像检测，复用现有低延迟 AprilTag 管线。

## 代码结构

```text
bunker_tag_follower/
  policy.py          # 无 ROS 依赖：坐标约定、跟随规则、限速和看门狗
  follower_node.py   # 最新检测 + 同时刻 TF -> 候选命令/预览/状态
  guard_node.py      # 独立进程：人工使能、失联停车、加速度限制
config/follow.yaml   # 距离、速度、置信度、超时等参数
launch/follow.launch.py
scripts/             # 使用系统 Python 的 ROS 可执行入口
test/                # 控制边界及 ROS 管线验证
```

数据流：

```text
Astra -> apriltag_ros -> /apriltag/detections + /tf
                                  |
                            tag_follower
                           /             \
       /follow/cmd_vel_preview          /follow/candidate (TwistStamped)
                                                    |
                                        follow_velocity_guard
                                                    |
                默认 /follow/cmd_vel_guarded_preview；明确启用硬件时才是 /cmd_vel
```

`/follow/state` 和 `/follow/guard_state` 输出控制/保护状态；
`/follow/target` 是目标 PoseStamped，可在 RViz 以 Pose 显示。
所有预览输出均不连接底盘。现有相机、检测、叠加画面、底盘驱动仍由各自终端管理，本包不会重复启动它们。

## 编译

```bash
cd ~/agx_ws
./scripts/build_humble.sh --packages-select bunker_tag_follower
source ~/agx_ws/setup_humble.bash
```

## 当前阶段：手持相机模拟

手持时相机朝向不等于小车朝向，所以只能验证“假设车头沿镜头方向”的控制意图。
不要用手持相机的坐标直接驱动车辆，也不要发布虚假的 base_link 静态变换。

分别启动（已经运行的无需重复启动）：

```bash
# 终端一：相机
source ~/agx_ws/setup_humble.bash
ros2 launch astra_camera astra_pro.launch.xml

# 终端二：标记检测，尺寸 15 cm
~/agx_ws/scripts/start_apriltag.sh 0.15

# 终端三：跟随预览
~/agx_ws/scripts/start_tag_follow.sh
```

手持模式将 optical.z 映射为前方、-optical.x 映射为左方，忽略高度，不声称这是实际车体坐标。
默认参数：保持 0.5 m，距离死区 0.08 m，最小 0.35 m，最大 4 m；
最高前进 0.03 m/s、转向 0.1 rad/s；偏角超过约 14 度先转向；不倒车、不丢失后原地搜索。
目标小于保持距离时停住（包括转向），避免近距离调整造成扫碰。
1 帧有效标记后才开始输出控制意图；无 ID 0、置信度低、TF 无效或观测超过 0.25 s 时输出零速度。

观察：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic echo /follow/state
ros2 topic echo /follow/cmd_vel_preview
```

实验：

- 标记在画面正中且距镜头大于约 0.58 m：linear.x > 0。
- 标记明显移到画面左侧：angular.z > 0，先转向；右侧相反。
- 距离约 0.5 m 或更近：速度为零。
- 遮住/拿走标记：速度归零；恢复后重新累计有效帧。
- 这里输出“期望速度”，不是实际车辆已执行的速度。

查看经保护后的预览（看门狗默认禁用，需显式使能）：

```bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /follow/cmd_vel_guarded_preview
```

关闭使能：

```bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: false}'
```

即使使能，默认模式也不会发布 /cmd_vel。跟随进程卡住/退出超过 0.3 s 后，独立 guard 持续输出零速度并锁回禁用；恢复后需重新使能。
时间检查同时使用源消息 ROS 时间戳和本机单调时钟，重复旧时间戳不能维持运动。

## 相机固定后的实车接入条件

1. 固定相机，测量安装位置和姿态，发布真实的 base_link 到相机根坐标系的 TF。不要重复发布相机驱动已有的内部 optical 变换。
2. 先以 `mode:=mounted hardware:=false` 验证实际车体坐标下的方向/距离。
3. 实车模式还要求独立障碍/人工监控模块持续发布 `/follow/path_clear`（std_msgs/Bool，true 且周期小于 0.3 s）。本包提供这一互锁接口及可选 depth_clearance 节点，已加入保守深度近障监控，尚未实现完整车体碰撞检测；不要用常量 true 冒充障碍检查。
4. 确认只有一个模块向 /cmd_vel 发布运动命令，清空其他导航/遥控速度源；准备遥控器和急停。
5. 完成以上验证后使用 `mode:=mounted hardware:=true`，仍需 /follow/enable 显式使能。

```bash
# 相机固定、外参和路径互锁已验证后才使用：
~/agx_ws/scripts/start_tag_follow.sh mode:=mounted hardware:=false
# 现场条件满足后可将 hardware 改为 true；不要同时启动两套跟随节点。
```

手持 + hardware=true 会被 launch 和 guard 两处拒绝。实车模式缺少外参、有效观测或路径互锁时不能产生运动。
独立 guard 能保护 follower 故障，但不能保护 guard 自身崩溃、整机断电或通信故障；仍需确认底盘自身通信超时停车及物理急停。

## 调参和测试

编辑 `config/follow.yaml` 后重启跟随；运行时参数修改不会热更新算法。
两处 max_linear/max_angular 分别是控制器目标限速和 guard 最终限速。
不要同时大幅修改两者。手持验证无需增加速度。

```bash
# 单元测试
cd ~/agx_ws
PYTHONPATH=src/bunker_tag_follower /usr/bin/python3 -m pytest src/bunker_tag_follower/test/test_policy.py -q

# ROS 集成测试：独立 ROS domain，合成观测，不接实车
source ~/agx_ws/setup_humble.bash
/usr/bin/python3 src/bunker_tag_follower/test/check_ros_pipeline.py
```

参考阅读：本机 `~/Robot-QR-code/README.md`，
[AprilTag ROS 2](https://github.com/christianrauch/apriltag_ros)。


已验证：27 项策略/深度测试通过；独立 ROS domain 中的进程测试覆盖转向符号、距离保持、遮挡、手动使能、强制终止 follower 后的停车，以及安装模式的外参缺失/补齐。
测试未向实际底盘发送运动指令。底盘运动、制动距离、真实安装变换和避障仍需现场验证。
