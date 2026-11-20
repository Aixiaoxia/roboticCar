# Bunker + Astra + AprilTag：ROS 2 Humble 完整使用指南

适用工作空间：`/home/aixiaoxiao/agx_ws`。整理日期：2026-10-21。

当前硬件是 Bunker 系列底盘、Astra Pro 相机；标记为 tag36h11 家族的 ID 0，黑色外边框实测边长 15 cm。
**当前已接入固定相机的低速跟随通道。相机居中、镜头水平朝前时，可按第 0 节进行现场操作；启用前必须确认状态和停车手段。**



## 当前速度更新

前进速度上限现已调到 **1.0 m/s**，控制器和保护节点的 `max_linear` 均已同步修改。
转向上限仍为 0.1 rad/s，前进加速度现为 0.5 m/s²，所以起步不会立刻达到最高速度，实际速度也受目标距离影响。
重启总启动后生效，旧章节中的 0.03 m/s、0.06 m/s、0.15 m/s、0.3 m/s、0.5 m/s、0.75 m/s 为此前设置。

## 深度超时检查更新（当前配置）

按现场要求，`check_depth_timeout: false` 已写入深度监控配置，需重启生效。
深度图停止更新或时间戳过期后，不再因该项停车；节点会沿用最后一帧的检查结果。
从未收到有效深度图、图像格式错误、近物或全图有效率不足时仍不放行。
`/follow/depth_state` 中 `depth_fresh` 表示深度是否新鲜；`clear_from_cached_depth` 表示正在沿用旧的可通行结果。
这不代表当前前方没有障碍，沿用期间新出现的障碍可能无法被发现。
跟随目标/候选速度的时间戳检查、指令看门狗及路径监控进程心跳检查仍保留。
恢复此项时将 `check_depth_timeout` 改回 `true` 并重启。

## 最新现场阈值（以本节为准）

按现场要求，`config/follow.yaml` 中 `follow_depth_clearance` 当前设置：

```yaml
stop_distance: 0.2
minimum_valid_fraction: 0.2
minimum_tile_valid_fraction: 0.15
check_blind_sectors: false
check_depth_timeout: false
timeout: 0.2
```

即全图有效深度至少 20%，有效深度小于 0.2 m 时停车，分区检查关闭；深度超时检查已按现场要求关闭。
相机位于车体中间，0.2 m 是相机量到的距离，不代表车头的净空；本设置不能保证避免车体碰撞。
先停止/禁用并确认车停，再退出总启动并重新运行 `~/agx_ws/scripts/start_all.sh` 才生效。
下文较早流程中提到的 50% 和 0.6 m 是旧阈值，当前以本节和配置文件为准。

## 当前现场配置更新：关闭分区盲区检查

按现场要求，`follow_depth_clearance.ros__parameters.check_blind_sectors` 已设为 `false`。
重启总启动后，不再因单个分区有效率低于 15% 而报 `blind_sector`。
全图有效率 50%、0.6 m 近障及深度超时检查仍启用。
这会减弱对局部深度缺失的防护，不能据此认定缺失区域没有障碍；继续保持低速和遥控接管。
需要恢复时将 `check_blind_sectors` 设为 `true` 并重启。

## AprilTag 平面位置 KF 已接入

默认启用普通 KF，首帧直接初始化，随后平滑前后和左右坐标；仍按原有目标超时停车，不靠预测延长行驶。
详细配置、原始/滤波坐标字段及验证步骤见 [AprilTag KF 平面滤波使用说明](AprilTag_KF平面滤波使用说明.md)。

## 参数冲突检查

启动前可运行 `~/agx_ws/scripts/check_follow_config.sh`，只校验、不启动硬件。
总启动会自动先校验距离和控制参数。必须满足 minimum_distance < desired_distance < maximum_distance，且 desired_distance > minimum_distance + distance_deadband。
当前保持距离已恢复为 0.5 m，以满足最小距离 0.35 m 与死区 0.08 m 的约束。

## 统一参数入口

现在只需编辑 `~/agx_ws/follow_config.yaml`，前进/转向上限只写一处，由控制器和保护节点共用。
文件含中文注释，修改后停止并重启即可，无需编译。
详细字段及生效步骤见 [跟随参数配置说明](跟随参数配置说明.md)。

## 一键启动：推荐入口

现在不必分别打开六个终端。先结束之前单独启动的相机、检测、可视化、底盘和跟随节点，避免重复占用设备或重复发布命令。
如果旧跟随已经使能，先禁用，确认车停，再到旧启动终端 Ctrl+C：

```bash
source ~/agx_ws/setup_humble.bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: false}'
```

### 一个终端启动整套系统

相机固定在车体中间且水平朝前、CAN 已 UP 且 500000 时：

```bash
~/agx_ws/scripts/start_all.sh
```

等价 ROS 2 入口：

```bash
source ~/agx_ws/setup_humble.bash
ros2 launch bunker_tag_follower bringup.launch.py
```

统一启动：Astra Pro、AprilTag ID 0、黄色框与坐标轴叠加、Bunker 驱动、跟随算法、独立速度保护、深度近障监控和控制窗口。
默认沿用当前配置：15 cm 标记、居中水平朝前安装模型、Bunker Mini 参数、目标距离 0.5 m、最高前进 0.03 m/s。
**默认接通硬件速度通道，但启动后仍是禁用状态，不会自动使能运动。**

启动检查发现 CAN 缺失/未启用/波特率不符或同名核心节点已运行时，会报错退出。不会自动 sudo 修改 CAN，也不会自动杀掉其他进程。
如果 CAN 未 UP，先在操作终端运行：

```bash
sudo ip link set can0 up type can bitrate 500000
```

### 在控制窗口中操作

窗口包含已经画好方框和坐标轴的图像，以及目标状态、保护状态、深度状态。
不再需要另外打开 rqt 或多个 echo 终端。

1. 将 ID 0 标记放在相机前方约 1～1.5 m，确认画面中的框和坐标轴正常。
2. 确认目标状态为 following/aligning、深度检查通过，遥控接管人员就位。
3. 点击 **使能跟随**。请求成功且目标满足条件后，车辆会开始移动。
4. 随时点击 **停止 / 禁用** 停车。服务失败原因会显示在按钮上方。
5. 首次只测试数秒直行。0.6 m 近障保护可能早于 0.5 m 算法目标停车，不要为贴近而绕过保护。

窗口显示的速度是算法期望速度，不等于底盘已执行的实际速度。底盘实际速度仍以 `/bunker_status` 为准。
关闭控制窗口会请求禁用；相机等节点仍由启动终端管理。
正常退出时先点击停止，确认车辆静止，再到唯一的启动终端 Ctrl+C 关闭整套节点。
相机/底盘、跟随器、保护或深度监控等关键进程退出时，总 launch 会关闭相关整套启动；这不能替代底盘自身通信超时和物理急停。

### 常用可选参数

```bash
# 只做整套手持预览，不启动底盘，不向 /cmd_vel 发运动指令
~/agx_ws/scripts/start_all.sh mode:=handheld hardware:=false start_base:=false

# 相机已单独运行，复用它；其他相关旧节点仍要先关闭
~/agx_ws/scripts/start_all.sh start_camera:=false

# 无图形桌面，不打开控制窗口；另用 /follow/enable 服务操作
~/agx_ws/scripts/start_all.sh panel:=false

# 普通 Bunker，而非 Mini
~/agx_ws/scripts/start_all.sh is_bunker_mini:=false
```

还支持 `start_detector`、`start_overlay`、`start_follow`、`start_depth_monitor`、`tag_size`、`port_name` 和 `config`。
默认实车通道需要深度监控提供有效的路径心跳；关闭深度监控不会解除保护条件。
如果相机不是居中水平朝前，应使用测量好的真实 TF，并设置 `centered_forward_mount:=false`。
首次增加总启动和控制窗口后，重新编译一次：

```bash
cd ~/agx_ws
./scripts/build_humble.sh --packages-select bunker_tag_follower
```

可复现的总启动验证（独立 ROS domain，不接真实相机或底盘）：

```bash
cd ~/agx_ws
source ~/agx_ws/setup_humble.bash
/usr/bin/python3 src/bunker_tag_follower/test/check_bringup.py
```

后续第 0 节和第 5 节保留逐终端方式，供排错使用；与一键方式二选一，不要同时启动。

## 0. 当前固定相机：现场完整操作顺序

本节适用于用户已确认的安装条件：相机刚性固定在车体平面中心附近，镜头水平朝车头正前方，不会滑动。
`centered_forward_mount:=true` 仅做已确认安装条件下的平面坐标换算：前方=optical.z，左方=-optical.x；忽略 optical.y 的上下偏差。
它不发布虚假的三维安装 TF，也不意味着相机外参已完成标定。高度对这一平面模型不参与计算；俯仰、歪斜或平面位置明显偏移时不能使用。

### 0.1 当前距离及限速

用户要求的跟随目标已改为 **0.5 m**，距离死区为 0.08 m，算法正中目标超过约 0.58 m 才产生前进意图。
最高前进速度为 **0.03 m/s**，最高转向速度为 **0.1 rad/s**。
0.5 m 是相机平面中心到标记的目标距离，**不等于车头到人的净空**，不要让人贴近履带或车头。
深度近障停车阈值独立保留 **0.6 m**；检测到近物会先停车，因此实际可能无法到达算法设定的 0.5 m。
不要为了“更贴近”直接缩小深度保护阈值；先量车体中心到车头的尺寸和实际停止距离。

### 0.2 六个运行终端

已运行的节点不要重复启动。每个 ROS 检查终端先 `source ~/agx_ws/setup_humble.bash`。

终端一：

```bash
source ~/agx_ws/setup_humble.bash
ros2 launch astra_camera astra_pro.launch.xml
```

终端二：

```bash
~/agx_ws/scripts/start_apriltag.sh 0.15
```

终端三（可选可视化）：

```bash
~/agx_ws/scripts/start_apriltag_visualization.sh
```

在另一个查看终端打开 rqt，并选择 `/apriltag/image_overlay`：

```bash
source ~/agx_ws/setup_humble.bash
ros2 run rqt_image_view rqt_image_view
```

终端四：检查并启用 CAN。若已 UP、500000，不必重复配置；否则执行需 sudo 的配置：

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 500000
```

然后启动底盘，当前沿用先前验证的 Mini 参数：

```bash
~/agx_ws/scripts/start_bunker.sh port_name:=can0 is_bunker_mini:=true
```

终端五：先结束旧的手持/其他跟随进程，再启动固定相机硬件通道：

```bash
~/agx_ws/scripts/start_tag_follow.sh mode:=mounted hardware:=true centered_forward_mount:=true start_depth_monitor:=true
```

**此时仍未使能，保护节点只发送零速度。**
不要同时启动多套跟随、多个深度监控或其他 `/cmd_vel` 发布者。

终端六：使能前逐项查看，检查命令之间按 Ctrl+C 切换：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic echo /follow/state
```

把 ID 0 标记放在车前方约 1～1.5 m、尽量位于画面中央，不要用手或身体挡住相机。
应看到 `mode: mounted`、`centered_forward_mount: true`，以及 `following`/`aligning` 等有效状态。
如果是 `target_missing_or_rejected`，先确认 ID 0 进入镜头，不能靠 enable 解决未识别问题。

```bash
ros2 topic echo /follow/depth_state
```

应看到 `clear: true`、`reason: clear_in_depth_view`；若为 false，按下面的状态表处理。
深度为 true 只表示监控视野内未触发阈值，人员仍需检查车体两侧及前缘。

```bash
ros2 topic echo /bunker_status
ros2 topic info /cmd_vel --verbose
```

确认底盘反馈正常，只有 `follow_velocity_guard` 向 `/cmd_vel` 发布。
遥控接管人员就位，场地空旷后，在操作终端执行：

```bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: true}'
```

`success: true` 表示通过使能检查；**如果此时有有效远处目标，车辆会开始低速移动。**
首次只进行数秒直线跟随，随后立即禁用，不要一开始就绕车侧测试大角度转向。

### 0.3 随时停止

保留一个操作终端，随时执行：

```bash
source ~/agx_ws/setup_humble.bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: false}'
```

确认 `/cmd_vel` 为零且底盘反馈速度归零，再关闭节点。
异常时直接用遥控器或物理急停接管。不要只关掉图像窗口或停止 echo。
近障、深度无效/过期或指令中断导致 guard 禁用后，即使画面恢复也需重新 enable。
仅目标短暂丢失时 follower 输出零速度；若 guard 心跳未中断，它仍可能保持 enabled，目标恢复后会重新跟随。需要禁止恢复时主动 disable。

### 0.4 深度监控状态

| 状态 | 处理 |
|---|---|
| `near_obstacle` | 视野内有小于 0.6 m 的近物；移开遮挡或调整安全位置，不要强行降低阈值 |
| `insufficient_depth` | 有效深度少于 50%；检查强光、玻璃/反光、线缆、视野及传感器状态 |
| `blind_sector` | 某个分区有效数据过少；先解决盲区 |
| `stale_depth` / `depth_timeout` | 深度数据过期或停止；检查相机与时间戳 |
| `invalid_depth_image` | 图像编码或数据格式错误 |
| `clear_in_depth_view` | 当前视野内检查通过，仍需要现场监护 |

源文件：`src/bunker_tag_follower/bunker_tag_follower/depth_node.py` 和 `depth_policy.py`。
参数位于 `src/bunker_tag_follower/config/follow.yaml` 的 `follow_depth_clearance` 段。

## 1. 当前已经实现什么

- 相机图像及内参发布。
- AprilTag ID 0 检测和相对相机的三维坐标。
- 图像上的黄色边框、ID、红绿蓝坐标轴及位置文字。
- 根据距离和左右偏差计算跟随速度。
- 独立速度保护进程：人工使能、限速、加速度限制及指令失联停车。
- 手持预览与相机固定后的安装模式。

已通过 27 项策略/深度单元测试，以及独立 ROS domain 中的真实节点集成测试。
已连接实车底盘并读到零速度反馈；尚未验证实车跟随运动和制动距离。新增深度近障停车只覆盖相机有效视野，不能替代完整车身避障。
当前没有实现 Nav2 路径规划、绕障或遮挡后搜索目标。

## 2. 工作空间与代码位置

```text
agx_ws/
├── setup_humble.bash                    # 每个终端加载环境
├── scripts/
│   ├── build_humble.sh                  # 使用系统 Python 编译
│   ├── start_bunker.sh                  # 启动底盘
│   ├── start_apriltag.sh                # 启动标记检测，必须指定尺寸
│   ├── start_apriltag_visualization.sh  # 启动图像叠加
│   └── start_tag_follow.sh              # 启动跟随和独立速度保护
├── config/apriltag.rviz                 # 三维可视化配置
├── config/apriltag_36h11_id0.yaml        # 检测配置
├── launch/apriltag_astra.launch.py
└── src/
    ├── ugv_sdk/
    ├── bunker_ros2/
    ├── ros2_astra_camera/
    ├── apriltag_ros/                    # 本地最新帧检测修改版
    └── bunker_tag_follower/
        ├── bunker_tag_follower/
        │   ├── policy.py               # 独立控制规则和看门狗
        │   ├── follower_node.py        # 检测/TF 数据适配和候选速度
        │   └── guard_node.py           # 独立速度保护进程
        ├── config/follow.yaml          # 跟随参数
        ├── launch/follow.launch.py
        ├── test/
        └── README.md
```

原参考代码位于 `~/Robot-QR-code`。本项目参考其“目标位姿 -> 距离和偏角 -> 速度”的思路，使用 ROS 2 和 AprilTag 重新实现，没有直接搬运 ROS 1 nodelet/ViSP 控制代码。

## 3. 首次准备与编译

已经编译好且代码没有修改时，可以跳过编译。
若需重新构建核心底盘、检测和跟随包：

```bash
cd ~/agx_ws
./scripts/build_humble.sh --packages-up-to bunker_base apriltag_ros bunker_tag_follower --allow-overriding apriltag_ros
```

相机驱动本工作空间已编译。如修改过相机代码，再单独执行：

```bash
cd ~/agx_ws
./scripts/build_humble.sh --packages-up-to astra_camera
```

构建脚本使用系统 Python，避免 Conda base 环境影响 ROS 依赖。
每个执行 ROS 命令的新终端先运行：

```bash
source ~/agx_ws/setup_humble.bash
```

检查：

```bash
ros2 pkg prefix bunker_tag_follower
ros2 pkg prefix apriltag_ros
ros2 pkg prefix astra_camera
```

应指向本工作空间的 `install` 目录。AprilTag 检测应加载工作空间修改版，才能使用 `qos_profile: latest`。

## 4. 标记准备

- 家族：网页生成器中的 `tag36h11`；检测配置中对应 `36h11`。
- ID：`0`。
- 尺寸：黑色外边框边长 0.15 m，不包含外面的白边。
- 贴在平整硬板上，保留白边，避免弯折、反光和手指遮挡。
- 15 cm 应以打印后尺子测量为准，不能只使用打印文件中的设定值。

## 5. 按终端顺序启动手持预览

每个长时间运行的命令使用独立终端。已经运行的节点不要再启动一次。
**手持预览不需要启动 Bunker 驱动，也不需要连接 CAN。**

### 终端一：启动相机

```bash
source ~/agx_ws/setup_humble.bash
ros2 launch astra_camera astra_pro.launch.xml
```

保持运行。检查话题时另开终端执行：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic list -t
```

应包含：

```text
/camera/color/image_raw     sensor_msgs/msg/Image
/camera/color/camera_info   sensor_msgs/msg/CameraInfo
/camera/depth/image_raw     sensor_msgs/msg/Image
```

当前彩色图像验证分辨率为 640×480。检测使用彩色图像及其内参，不能误接深度相机的内参。

### 终端二：启动 AprilTag 检测

```bash
~/agx_ws/scripts/start_apriltag.sh 0.15
```

将标记正对相机，先在约 0.5～1 m 处检查识别，保证完整标记和白边在画面内。
另开检查终端：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic echo /apriltag/detections
```

正常应出现 `family: tag36h11`、`id: 0`。
没有目标时 `detections: []` 正常。这里按 Ctrl+C 只停止查看，不会停止检测节点。

查看标记坐标：

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame target_tag
```

相机光学坐标系：x 向右、y 向下、z 向前，单位米。
将标记放在镜头前实测约 1 m，检查 z 是否接近 1.0，并随移动更新。
TF 缓存可能保留丢失目标的最后位置，是否仍可见应以新检测结果为准。

### 终端三：启动边框与坐标轴叠加

```bash
~/agx_ws/scripts/start_apriltag_visualization.sh
```

### 终端四：打开画面

```bash
source ~/agx_ws/setup_humble.bash
ros2 run rqt_image_view rqt_image_view
```

在顶部下拉框选择：

```text
/apriltag/image_overlay
```

不要选择 `/camera/color/image_raw`，原始图像不会有绘制结果。
正常显示黄色方框、ID 0、红 X / 绿 Y / 蓝 Z 坐标轴，以及顶部位置文字。
标记轴长度默认 7.5 cm。顶部 camera x/y/z 是标记原点相对相机的位置。

可选：新终端打开三维坐标系：

```bash
source ~/agx_ws/setup_humble.bash
rviz2 -d ~/agx_ws/config/apriltag.rviz
```

Fixed Frame 为 `camera_color_optical_frame`，TF 中可以查看 `target_tag`。
这时还没有标定真实车体到相机的安装关系。

### 终端五：启动手持跟随预览

```bash
~/agx_ws/scripts/start_tag_follow.sh
```

等价于：

```bash
source ~/agx_ws/setup_humble.bash
ros2 launch bunker_tag_follower follow.launch.py mode:=handheld hardware:=false
```

该命令启动 `tag_follower` 和 `follow_velocity_guard` 两个独立进程。
手持模式假设车头朝向镜头正前方，以 optical.z 作为前方、-optical.x 作为左方。
手持相机移动本身会改变目标坐标；这只是在模拟控制意图，不是车体定位结果。

### 终端六：查看跟随状态和期望速度

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic echo /follow/state
```

需要看速度时，在检查终端按 Ctrl+C 后改为：

```bash
ros2 topic echo /follow/cmd_vel_preview
```

关键字段：`linear.x` 为前进速度（m/s），`angular.z` 为转向速度（rad/s）。
正转向代表向左，负转向代表向右。

## 6. 手持模拟验收步骤

让相机尽量水平、方向保持稳定，通过移动标记检查控制意图。
距离指目标在相机水平投影中的距离，标记明显偏侧时并非只看 z。

| 操作 | 预期状态/速度 |
|---|---|
| 正前方约 1.5～2 m | `following`，linear.x > 0 |
| 远处标记明显移到画面左侧 | `aligning`，linear.x=0，angular.z>0 |
| 远处标记明显移到画面右侧 | `aligning`，linear.x=0，angular.z<0 |
| 标记约 0.5 m 或更近 | `holding_distance` 或 `too_close`，两轴均为零 |
| 遮住或拿走标记 | 两轴归零，不继续追踪旧位置 |
| 将标记重新放回 | 1 帧有效观测后恢复计算 |

当前默认参数：

| 参数 | 值 |
|---|---:|
| 保持距离 | 0.5 m |
| 距离死区 | 0.08 m |
| 最小距离 | 0.35 m |
| 最大跟随距离 | 4.0 m |
| 最大前进速度 | 0.03 m/s |
| 最大转向速度 | 0.1 rad/s |
| 先转向阈值 | 0.25 rad，约 14° |
| 观测超时 | 0.25 s |
| 连续有效帧数 | 1 |

正中目标距离大于约 0.58 m 才输出前进，标记放得太近时零速度是设计行为。
第一版不倒车、不自动寻找丢失目标。测试以上步骤不需要启动底盘。

## 7. 验证独立速度保护

`/follow/cmd_vel_preview` 是算法输出，不经过加速度保护。
`/follow/cmd_vel_guarded_preview` 是保护后的输出，启动时禁用，因此先看到零速度正常。

在新终端中：

```bash
source ~/agx_ws/setup_humble.bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: true}'
```

返回 `success: true` 后查看：

```bash
ros2 topic echo /follow/cmd_vel_guarded_preview
```

目标在前方较远处时，前进速度应缓慢增加，仍不超过 0.03 m/s。
无目标、近距离或目标丢失时归零。即使使能，当前预览模式也不会连接 `/cmd_vel`。

关闭使能：

```bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: false}'
```

看门狗独立于跟随进程；跟随进程不再发送指令超过 0.3 s 后，保护输出归零并锁回禁用，恢复后需要重新使能。
目标消息和 TF 有效性在跟随进程中检查；候选命令的新鲜度及心跳在保护进程中再次检查。
不要为了验收而在实车上随意 kill 节点；下文的隔离集成测试已覆盖强制崩溃行为。

## 8. 正常退出顺序

当前手持预览：

1. 如已使能，先调用 `/follow/enable`，设为 false。
2. 确认 `/follow/cmd_vel_guarded_preview` 为零。
3. 在跟随启动终端按 Ctrl+C，退出跟随和保护进程。
4. 退出 rqt/RViz 和可视化节点。
5. 退出 AprilTag 检测。
6. 最后退出相机节点，释放 USB 设备。

退出 `ros2 topic echo` 或 `tf2_echo` 只停止查看，不会停止被查看的节点。
不要把关闭查看窗口等同于停止控制。

## 9. 相机固定后的接入流程

**这部分是后续实车接入步骤，不适用于当前手持模拟。**
当前包已提供底盘输出和深度近障监控。实车运动仍需现场验证，不能当作完整避障系统。相机居中、水平朝前时可使用第 0 节的平面安装模型；其他安装方式必须测量外参。

### 9.1 固定并测量相机安装关系

将相机刚性固定，测量其相对车体 `base_link` 的平移和旋转。
当前相机默认根坐标系是 `camera_link`，驱动已经发布内部光学坐标变换。
需要补充 `base_link -> camera_link`，不要重复给 optical frame 再指定另一个父坐标系。

静态变换发布命令的模板如下；尖括号中的值必须替换为实际测量结果，不能原样运行：

```text
ros2 run tf2_ros static_transform_publisher \
  --x <米> --y <米> --z <米> \
  --roll <弧度> --pitch <弧度> --yaw <弧度> \
  --frame-id base_link --child-frame-id camera_link
```

车体轴约定：x 前、y 左、z 上。角度表示 camera_link 相对 base_link 的姿态，不能因为镜头朝前就直接猜 optical 旋转为零。
如相机命名已改动，应同步调整实际 frame 名称和跟随配置。

检查：

```bash
source ~/agx_ws/setup_humble.bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

### 9.2 先以安装模式预览

停止先前的手持跟随节点，再启动：

```bash
~/agx_ws/scripts/start_tag_follow.sh mode:=mounted hardware:=false
```

重新检查转向符号、实际车体到标记的距离和目标丢失停车。
没有完整 TF 链时，应保持零速度，而不是绕过 TF 检查。

### 9.3 接入独立路径监控

实车模式必须有独立模块持续发布：

```text
/follow/path_clear    std_msgs/msg/Bool
```

只有收到 true 且消息接收间隔不超过 0.3 s，保护节点才允许使能或继续输出。
遇到障碍、深度数据无效或监控停止时，应为 false 或不再发送，保护节点将停车并禁用。

本工作空间现已加入 Astra 深度近障监控（`start_depth_monitor:=true`）。
它检查横向全宽、纵向 10%～85% 的深度区域；默认近物阈值 0.6 m、有效像素至少 50%，并检查分区盲区。深度停止更新超过 0.2 s 时输出 false。
这是保守的视野内停车辅助，不能覆盖车侧、履带前缘、低矮物、落差或转向扫过区域；必须有人遥控接管。
不要发布固定 true 绕过检查，也不要把可见 AprilTag 当作路径没有障碍。

### 9.4 CAN 与 Bunker 底盘

接好 CAN，按对应型号手册设置遥控器控制模式，准备遥控接管和急停。
先检查接口：

```bash
ip -details link show can0
```

若已 UP 且 bitrate 500000，不必重复配置。
若接口存在但未启用：

```bash
sudo ip link set can0 up type can bitrate 500000
```

`gs_usb` 型适配器才需 `sudo modprobe gs_usb`；先前此电脑识别到的适配器使用 `pcan_usb`。
可选被动收包（先安装 can-utils）：

```bash
candump can0
```

按 Ctrl+C 结束收包。普通 Bunker 启动：

```bash
~/agx_ws/scripts/start_bunker.sh port_name:=can0
```

确认为 Bunker Mini 才使用：

```bash
~/agx_ws/scripts/start_bunker.sh port_name:=can0 is_bunker_mini:=true
```

不要启动 Scout 驱动。先检查 `/bunker_status` 有持续有效反馈。

### 9.5 最后启用硬件输出

必须先满足：相机固定、真实外参已验证、安装模式预览通过、独立路径监控运行、急停可用、测试区域空旷。
停止其他跟随/导航/测试速度发布进程，确保只有保护节点作为本次测试的 `/cmd_vel` 发布者。

停止安装模式预览后启动：

```bash
~/agx_ws/scripts/start_tag_follow.sh mode:=mounted hardware:=true
```

此时保护节点持续发布零速度，启动不等于使能。
检查图结构：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /follow/guard_state
```

确认条件满足后，再在独立终端调用：

```bash
ros2 service call /follow/enable std_srvs/srv/SetBool '{data: true}'
```

以响应中的 success 为准。返回 `path_not_confirmed_clear` 表示路径监控没有满足要求；不能靠修改模式绕过。

停止：先 `/follow/enable` false，确认 `/cmd_vel` 为零且车辆已停，再退出跟随节点及底盘节点。
反应异常时优先使用遥控或物理急停，不要等待 ROS 命令。
独立 guard 不保护自身进程崩溃、整机断电或通信故障，实车还必须验证底盘自身的超时停车机制。

## 10. 主要话题和服务

| 名称 | 类型 | 用途 |
|---|---|---|
| `/camera/color/image_raw` | Image | 原始彩色画面 |
| `/camera/color/camera_info` | CameraInfo | 彩色图像内参 |
| `/apriltag/detections` | AprilTagDetectionArray | 新检测的 ID、角点和置信度 |
| `/tf` | TFMessage | 相机/标记等坐标变换 |
| `/apriltag/image_overlay` | Image | 方框和坐标轴叠加图 |
| `/follow/target` | PoseStamped | 接受的目标位姿 |
| `/follow/state` | String，JSON | 模式、控制状态、距离和速度 |
| `/follow/cmd_vel_preview` | Twist | 未经独立保护的算法预览 |
| `/follow/candidate` | TwistStamped | 带源时间戳的内部候选命令 |
| `/follow/cmd_vel_guarded_preview` | Twist | 预览模式的保护输出 |
| `/follow/guard_state` | String | 保护状态 |
| `/follow/path_clear` | Bool | 深度监控或经验证的独立路径互锁 |
| `/follow/depth_state` | String，JSON | 深度监控结果、有效率及最近距离 |
| `/cmd_vel` | Twist | 仅硬件模式下的保护输出 |
| `/follow/enable` | SetBool 服务 | 人工使能/停用 |

## 11. 状态与排错

| 提示/现象 | 原因和处理 |
|---|---|
| `Package ... not found` | 在报错终端 source 工作空间；必要时重新编译 |
| 相机 `Resource busy` | 已有相机进程占用；使用现有节点，或到原终端 Ctrl+C 后再启动 |
| `BunkerStatus is invalid` | 当前终端未加载自定义消息，重新 source |
| 只有相机画面，没有框 | 确认选中 `/apriltag/image_overlay`；检查检测/叠加节点运行 |
| 检测数组为空 | 检查家族、ID、光线、反光、白边、完整入镜和距离 |
| 有框、坐标轴不更新 | 检查匹配 TF、相机内参及时间戳，不要使用缓存坐标判断新检测 |
| `waiting_for_target` | 暂未收到目标 |
| `acquiring` | 等待累计足够的新有效帧 |
| `target_missing_or_rejected` | 标记不见、ID/家族/置信度不符，或图像 frame 不符 |
| `waiting_for_matching_tf` | 对应时间的 TF 尚不可用，安装模式还需真实外参 |
| `stale_detection` / `target_timeout` / `observation_timeout` | 数据过期，先查相机、检测和时间戳 |
| `holding_distance` / `too_close` | 距离太近，不前进是预期行为 |
| `out_of_range` | 超出最大距离，先靠近并确认测距准确 |
| `aligning` | 先转向对准，当前不前进 |
| 预览非零，但保护预览为零 | 默认未使能，检查 enable 响应和 guard_state |
| `no_command` | 保护节点还未收到合法候选指令 |
| `disabled` | 尚未使能或已手动禁用 |
| `stale_command_stamp` / `command_watchdog` | 候选指令过期/停止；恢复后需重新使能 |
| `path_not_confirmed_clear` | 硬件模式缺少新鲜且为 true 的路径互锁 |
| `Handheld mode cannot drive Bunker` | 手持模式禁止硬件输出，当前应继续预览 |

检查节点：

```bash
source ~/agx_ws/setup_humble.bash
ros2 node list
```

完整预览链路应包含 `/camera/camera`、`/apriltag`、`/apriltag_overlay`、`/tag_follower`、`/follow_velocity_guard`。
相机和检测入口已采用低延迟策略，旧图像会被丢弃，而不是逐帧追赶。
检测入口队列只保留最新一帧，超过 100 ms 的图像跳过；当前帧计算仍会正常完成。
不要通过复用旧目标或延长超时来掩盖卡顿。

## 12. 配置、标定和测试

跟随参数在：

```text
~/agx_ws/src/bunker_tag_follower/config/follow.yaml
```

修改后重启跟随节点。运行时 `ros2 param set` 不会让本包已经加载的控制参数自动生效。
控制器和保护节点分别有 max_linear/max_angular；最终输出还受保护节点限幅，不要同时大幅提高两者。
改变 ID 时，要同步修改检测配置中的 tag.ids/tag.frames 以及跟随配置 tag_id/tag_frame；目前可视化主要针对 ID 0。

检测距离依赖标记实际尺寸和正确相机内参。当前相机发布 D=0 不等于镜头已完成精确标定。
如果 1 m 实测距离偏差明显，先检查打印黑框尺寸，再检查标定。
之后如果加载非零畸变参数，应接入校正后的图像和相应内参，不能继续把未校正图像当作 image_rect 使用。

单元测试：

```bash
cd ~/agx_ws
PYTHONPATH=src/bunker_tag_follower /usr/bin/python3 -m pytest src/bunker_tag_follower/test/test_policy.py -q
```

ROS 集成测试：

```bash
cd ~/agx_ws
source ~/agx_ws/setup_humble.bash
/usr/bin/python3 src/bunker_tag_follower/test/check_ros_pipeline.py
/usr/bin/python3 src/bunker_tag_follower/test/check_depth_pipeline.py
```

集成测试使用独立的 ROS_DOMAIN_ID=177、仅本机通信和合成数据，不连接当前实车图。
请勿让实际硬件节点使用该测试 domain。测试会自行启停测试节点。
覆盖转向符号、保持距离、目标遮挡、使能、强制终止 follower 后的停车、安装模式的 TF 缺失/补齐，且检查没有 `/cmd_vel` 发布者。

## 13. 每次使用速查

当前手持阶段，顺序为：

1. 启动相机。
2. `~/agx_ws/scripts/start_apriltag.sh 0.15`。
3. `~/agx_ws/scripts/start_apriltag_visualization.sh`。
4. rqt 选择 `/apriltag/image_overlay`。
5. `~/agx_ws/scripts/start_tag_follow.sh`。
6. 查看 `/follow/state` 和 `/follow/cmd_vel_preview`，按第 6 节测试。
7. 如需验证保护后的预览，再调用 `/follow/enable`。
8. 退出时先禁用保护，再按第 8 节关闭节点。

以上手持流程不会让车辆自动移动。固定相机的现场操作使用第 0 节；其他安装姿态仍需第 9 节的真实外参流程。


## 分区深度阈值更新

分区有效深度阈值已按现场要求从 25% 改为 **15%**。
配置字段是 `follow_depth_clearance.ros__parameters.minimum_tile_valid_fraction: 0.15`，修改后重启总启动生效。
全图有效深度阈值仍为 50%，近障阈值仍为 0.6 m；本次降低阈值并不保证稀疏区域没有障碍。


## 首次识别后的起步等待

当前 acquire_frames=1：首个有效检测及对应时刻 TF 就能产生跟随指令，不再等待连续 3 帧。
控制与速度保护以 20 Hz 更新，保护从零按加速度限制逐渐提升速度。
新加速度 0.1 m/s² 下，约 0.5 s 到 0.05 m/s、1 s 到 0.1 m/s，达到 0.5 m/s 上限约需 5 s。
实际能否开始滚动取决于底盘低速响应，不保证第一个非零速度就能克服静摩擦。
目标偏角超过 0.25 rad 时只转向，目标距离小于约 0.58 m 时不前进。
线速度还受比例控制影响：目标居中时约为 min(0.5, 0.2 × (距离 - 0.58))；提高上限不代表近距离也固定跑 0.5 m/s。
当前连续帧确认已改为 1 帧；增益和转向优先规则保持原设置。
