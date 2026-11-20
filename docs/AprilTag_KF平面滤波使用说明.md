# AprilTag 普通卡尔曼滤波（KF）使用说明

## 当前实现

滤波位于已确认的 AprilTag 平面位置与跟随控制之间，不重复处理相机图像。

```text
AprilTag ID 0 + 对应时间的 TF
        ↓
坐标转换：前后、左右
        ↓
普通 KF：估计平面位置及相对速度
        ↓
原始位置停车约束 + 跟随控制
        ↓
现有独立速度保护 → 底盘
```

状态向量为 `[前后位置, 左右位置, 前后相对速度, 左右相对速度]`。
采用短时间匀速模型；每次收到不同时间戳的有效测量，先预测到该测量时刻，再校正。
高度不参与该平面 KF。相对速度包含机器人自身运动带来的影响，不是人的世界坐标速度。
估计速度目前仅用于滤波内部预测及诊断，没有额外增加速度前馈控制。

## 启动

已编译安装。当前程序不会热更新，新配置需要重启。
先点击“停止 / 禁用”、确认车辆停止，再退出旧总启动并运行：

```bash
~/agx_ws/scripts/start_all.sh
```

相机安装模型、使能方式和其他阈值继续使用当前工作空间配置。
本次 KF 改动没有提高速度、缩短跟随距离或解除其他现有停车条件。
控制窗口的“前方”会显示为“KF前方”，表示用于控制的平滑后位置。
黄色标记框与三轴仍来自图像检测/原始 TF，因此能反映原始检测变化。

## 首帧与目标丢失

- 首帧有效测量直接初始化位置，初始相对速度为零；不额外等待多帧。
- 仍使用 `acquire_frames: 1`；需要置信度、时间戳和 TF 检查通过。
- 收到空检测、错误 ID/家族/低置信度时清空目标并重置 KF。
- 检测停止更新，仍在现有 observation_timeout（0.25 s）后归零并重置。
- 不会只靠预测持续追踪丢失目标，不会刷新旧观测时间戳来延长运动。
- 观测间隔超过超时时间，恢复后的新测量直接重新初始化，不沿用旧速度。
- 原始位置触发距离过近、距离保持、超出范围或目标在身后时，停车约束优先于滤波。
- 原始偏角触发“先对准”时，仍优先执行原有先转向规则，不因滤波让大偏角目标继续前进。

KF 能减少坐标抖动，但不能识别所有错误测量、替代避障或弥补错误相机标定。
单帧检测已被允许；KF 首次初始化也会接受该首帧，所以首帧误检仍可能触发控制意图。

## 配置与调参

文件：`~/agx_ws/src/bunker_tag_follower/config/follow.yaml`。

```yaml
tag_follower:
  ros__parameters:
    kf:
      enabled: true
      position_std: 0.04
      acceleration_std: 2.0
      initial_velocity_std: 1.0
```

| 参数 | 含义 | 调整影响 |
|---|---|---|
| enabled | 是否使用 KF | false 时恢复原始平面位置控制 |
| position_std | 测量位置噪声标准差，米 | 增大更平滑，但可能更滞后；减小更贴近原始测量 |
| acceleration_std | 相对加速度过程噪声标准差，m/s² | 增大更容易跟上转向/速度变化，减小更相信匀速模型 |
| initial_velocity_std | 首次初始化的速度不确定性，m/s | 初速度仍为零；此值影响后续测速适应速度 |

上述值是初始调试值，需要结合相机标定、识别距离和车体运动调整，不是通用最优参数。
加速度噪声不是底盘加速度限制；底盘限制仍由 `follow_velocity_guard.linear_accel` 决定。
不要为了消除抖动盲目增大平滑程度；目标突然靠近和机器人转动时应检查响应滞后。

## 查看原始与滤波结果

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic echo /follow/state
```

新增字段：

| 字段 | 含义 |
|---|---|
| kf_enabled | KF 是否启用 |
| raw_forward_m / raw_left_m | 本次有效测量的原始平面位置 |
| forward_m / left_m | KF 后的平面位置，用于正常跟随控制 |
| relative_forward_velocity_mps / relative_left_velocity_mps | KF 估计的相对速度 |

目标失效后原始位置和速度估计字段清空。停车约束可能直接使用原始位置，因此不能只看滤波后的位置判断为什么停住。

可视化话题：

- `/follow/target`：原始 PoseStamped。
- `/follow/filtered_target`：平滑后 PoseStamped。上下位置和姿态仍沿用原始测量。

两个话题都使用原始观测时间戳与对应坐标系；居中光学模型下仍在相机光学坐标系，不能直接误认为三维 base_link 位姿。
可在 RViz 添加两个 Pose 显示，分别设置不同颜色进行比较。

## 建议验证动作

1. 默认未使能，先固定相机、保持标记静止，观察原始和滤波位置的抖动差异。
2. 缓慢左右移动标记，检查方向正确且没有明显拖尾。
3. 快速把标记移近，确认原始距离停车约束立即归零。
4. 遮挡标记、停止检测输入，确认速度归零；重新出现后不延续旧目标速度。
5. 以上通过后，再在有人遥控接管的空旷场地短时低速验证。

## 代码与测试

- 算法：`src/bunker_tag_follower/bunker_tag_follower/kalman.py`。
- ROS 接入：`src/bunker_tag_follower/bunker_tag_follower/follower_node.py`。
- 数学与边界测试：`src/bunker_tag_follower/test/test_kalman.py`。
- 管线测试：`src/bunker_tag_follower/test/check_ros_pipeline.py`。

```bash
cd ~/agx_ws
PYTHONPATH=src/bunker_tag_follower /usr/bin/python3 -m pytest src/bunker_tag_follower/test/test_kalman.py -q
source ~/agx_ws/setup_humble.bash
/usr/bin/python3 src/bunker_tag_follower/test/check_ros_pipeline.py
```

管线测试使用独立 ROS domain 和合成数据，不连接真实硬件。
