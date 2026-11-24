# Bunker + Astra 相机外参标定流程

本文记录如何采集 Bunker 底盘、Astra 相机和固定 AprilTag 板的数据，并离线求解 `base_link` 到相机的外参。

## 1. 标定关系和准备

AprilTag 板固定在场地中，不能随小车移动；相机刚性安装在小车上。小车在不同位置和朝向观测同一个 Tag 0。采集器记录同一观测时刻的：

- `odom → base_link`：底盘里程计位姿；
- `camera_color_optical_frame → target_tag`：由 Tag 四角点和相机内参直接计算的位姿。

离线求解满足以下关系的 `base_link → camera` 外参，以及固定 Tag 在 `odom` 中的位姿：

```text
odom_T_base × base_T_camera × camera_T_tag = odom_T_tag
```

准备固定且平整的 Tag 板，量出 Tag **黑色外框**边长（米）。板不能晃动；相机安装不能松动。采集时遥控器和急停应可用，操作者应能立即停车。采集器不会发送运动指令。

## 2. 编译采集器和求解器

在工作空间终端执行：

```bash
cd ~/agx_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bunker_tag_follower
source ~/agx_ws/setup_humble.bash
```

每个后续终端都执行 `source ~/agx_ws/setup_humble.bash`。底盘、相机、AprilTag 和采集器必须处于**同一个 ROS domain**。不要把实车节点和另一个 ROS domain 中的仿真节点混在一起。

## 3. 启动底盘

先检查 CAN：

```bash
ip -details link show can0
```

若 `can0` 已经是 UP 且 bitrate 为 `500000`，无需重复配置；否则按本机 CAN 适配器配置：

```bash
sudo ip link set can0 up type can bitrate 500000
```

普通 Bunker：

```bash
source ~/agx_ws/setup_humble.bash
~/agx_ws/scripts/start_bunker.sh port_name:=can0
```

Bunker Mini 才添加 `is_bunker_mini:=true`：

```bash
source ~/agx_ws/setup_humble.bash
~/agx_ws/scripts/start_bunker.sh port_name:=can0 is_bunker_mini:=true
```

另开终端检查底盘里程计和 TF：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

`/odom` 应持续发布，TF 应持续显示。若 `odom`、`base_link` 名称和实际配置不同，采集器需要使用对应 frame 参数。不要用伪造的静态 `odom → base_link` 代替里程计。

## 4. 启动 Astra 和 AprilTag

新终端启动 Astra Pro：

```bash
source ~/agx_ws/setup_humble.bash
ros2 launch astra_camera astra_pro.launch.xml
```

另开终端启动 Tag 36h11 检测。例子中的 `0.15` 仅适用于黑色外框实测为 15 cm 的 Tag：

```bash
source ~/agx_ws/setup_humble.bash
~/agx_ws/scripts/start_apriltag.sh 0.15
```

相机彩色图像默认是 `/camera/color/image_raw`，检测话题是 `/apriltag/detections`，目标 ID 为 0。检查话题：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /apriltag/detections
```

在 Tag 进入视野后，检测消息中应能看到 ID 0。Tag 尺寸填写错误会直接影响 PnP 距离和最终外参。

## 5. 采集数据

另开终端启动采集器。输出文件建议使用新名字，避免把以前的采集追加进来；`tag_size` 必须和 AprilTag 启动参数一致：

```bash
source ~/agx_ws/setup_humble.bash
ros2 run bunker_tag_follower record_extrinsic --ros-args \
  -p tag_size:=0.15 \
  -p output:=~/agx_ws/camera_extrinsic_run1.jsonl
```

采集窗口中，绿色多边形表示 Tag 0 已检测。小车移动到一个位置后先**停稳**，确认 Tag 框可见，再点一下图像窗口使其获得键盘焦点并按 Enter。成功时终端和画面会显示 `已记录第 N 组`；未记录时会显示原因。按 `q` 或 Esc 退出。

采集建议：

- 至少记录 20 组有效样本；覆盖近、中、远距离和图像内不同位置。
- 包含明显的前后、左右移动以及左右转向；不要只沿直线走，也不要只原地转。
- 每次按 Enter 前停稳，保持 Tag 板不动，避免里程计时间偏差和运动模糊。
- 只把打印 `已记录第 N 组` 的样本计入有效数量。若提示 odom 与 Tag 时间差过大，先检查 `/odom` 和 TF 是否持续更新。
- 采集器将 `camera_link → camera_color_optical_frame` 内部 TF 一并存入样本；若驱动没有发布该 TF，求解器只能输出 optical frame 外参。

采集器在 TF 查找时使用独立监听线程。若仍反复报告时间差超过 0.25 秒，不要放大门限强行采集；检查各节点是否在同一 ROS domain、系统时间是否一致，以及 `/odom` 和 `/tf` 时间戳是否持续更新。

## 6. 离线求解

退出采集器后，在工作空间终端运行求解器：

```bash
source ~/agx_ws/setup_humble.bash
ros2 run bunker_tag_follower solve_extrinsic \
  ~/agx_ws/camera_extrinsic_run1.jsonl \
  --output ~/agx_ws/camera_extrinsic_result.json
```

求解器使用多组位姿联合优化固定 Tag 位姿和 `base_link → camera` 外参。默认至少需要 10 组样本，并会排除 odom 与 Tag 时间差大于 0.10 秒的样本；建议实际采集 20 组以上。它会输出：

- `base_T_camera_optical`：`base_link → camera_color_optical_frame`；
- `base_T_camera_link`：利用 Astra 内部 TF 换算出的 `base_link → camera_link`（内部 TF 可用时）；
- 平移和旋转残差 RMS、参与求解的样本数量及结果 JSON 文件。

结果还包含残差 P95、优化雅可比的有效秩和条件数，以及 `quality`（`good` / `review` / `poor`）。这些是拟合和可观测性诊断，**不是成功概率或统计置信度百分数**。若垂向不可观测、矩阵秩不足、条件数过高或残差超限，终端会标 `poor` 并以非零状态退出；即使 JSON 中有数值，也不要应用。Bunker 只在地面平面移动且 Tag 世界高度未知时，相机绝对高度通常不可观测；需提供独立的高度测量/约束后才能确认完整六自由度外参。

若有效样本不足，先按提示补采。求解器会拒绝最大平移范围小于 0.25 m 或总转角小于 20° 的数据。若残差较大或优化器未收敛，不要直接把结果写入 TF；先检查 Tag 实测尺寸、相机内参、标定板固定情况、里程计漂移，并补采包含多方向移动和转向的数据。

## 7. 应用和复核外参

结果 JSON 中的 `base_T_camera_link.translation_m` 和 `rpy_rad` 是写入静态 TF 的平移（米）和 RPY（弧度）。替换 [start_bunker_camera_tf.sh](../scripts/start_bunker_camera_tf.sh) 中的初始值，然后重启静态 TF 发布器。不要同时保留旧值和新值两个 `base_link → camera_link` 发布者。

用以下命令复核最终变换：

```bash
source ~/agx_ws/setup_humble.bash
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

再把标定板放到未参与求解的位置和距离验证相机测得的 Tag 位姿。该方法依赖轮速里程计；底盘打滑、轮径误差、Tag 尺寸误差、内参误差或相机支架松动都会影响精度。标定完成后用 RTAB-Map 建图复核，但不能只凭地图视觉效果代替独立验证。

## 8. 常见提示

| 提示 | 含义和处理 |
|---|---|
| `当前画面没有同步的 AprilTag 检测` | Enter 时没有找到时间接近的检测帧。确认 Tag 框稳定可见、检测话题在更新，并在小车停稳后采集。 |
| `odom frame ... does not exist` | 底盘驱动未运行、ROS domain 不一致或 frame 名配置不一致。检查 `/odom` 和 `odom → base_link`。 |
| `odom 与 Tag 时间差 ... 超过 0.25s` | TF 时间未跟上或时间源不一致。不要强行放宽门限；检查 `/tf` 更新和各节点系统时钟。 |
| `未收到有效 camera_info 内参` | 相机内参话题未发布或投影矩阵无效。检查 `/camera/color/camera_info`。 |
| 求解时有效样本不足 | 增加有效采样，确认采集文件路径和时间差过滤条件。 |
