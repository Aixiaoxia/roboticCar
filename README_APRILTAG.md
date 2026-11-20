# Astra Pro + AprilTag（ROS 2 Humble）

## 当前验证结果

- 已安装 apriltag_ros 3.4.0。
- Astra Pro 彩色图像：/camera/color/image_raw，640×480、rgb8。
- 彩色内参：/camera/color/camera_info，时间戳与图像匹配。
- optical frame：camera_color_optical_frame。
- 当前 K：fx=fy=570.3422047415297，cx=319.5，cy=239.5。
- 当前 D 为五个零；这并不证明真实镜头没有畸变或已准确标定。
- 在独立测试话题发布合成 tag36h11 ID 0，检测成功，已知 0.375 m 距离的 TF 输出约 0.375 m。
- 实际相机接入后收到持续的检测数组（未放标记时为空，属正常）。
- 检测节点不发布 /cmd_vel，不控制底盘。

## 启动

若相机已经运行，不要重复启动；否则终端一：

```bash
source ~/agx_ws/setup_humble.bash
ros2 launch astra_camera astra_pro.launch.xml
```

终端二：

```bash
# 仅当打印后黑色外边框实测为 15 cm 时，使用 0.15。
~/agx_ws/scripts/start_apriltag.sh 0.15
```

尺寸为黑色外边框方形的实际边长，单位米，不含外侧白边。
脚本要求显式给出正数尺寸，无默认尺寸。打印 20 cm 则填写 0.20。
未打印前用手机/屏幕显示标记可以检查 ID，距离结果必须按屏幕上实际显示的黑色方框边长设定尺寸才能参考。

终端三：

```bash
source ~/agx_ws/setup_humble.bash
ros2 topic echo /apriltag/detections
ros2 run tf2_ros tf2_echo camera_color_optical_frame target_tag
```

配置家族为 36h11（图片网站对应 tag36h11），tag.ids=[0]，tag.frames=[target_tag]。
ID 0 的位姿通过 /tf 发布；检测数组包含 ID、角点及置信度等，不含三维 pose 字段。
配置中的 tag.ids 用于选择发布 TF 的标记；后续跟随代码也应显式筛选 ID 0，并检查检测时间，不能使用缓存 TF 认定目标仍可见。
光学坐标系：x 向右，y 向下，z 向前，不能直接当作 base_link 坐标。

## 原始图像与标定

apriltag_ros 输入名叫 image_rect，应使用经过校正的图像和对应投影矩阵 P。
当前驱动输出 D=0、R=I、P 左侧矩阵等于 K，因此配置直接接 image_raw：按当前发布的相机模型，校正是恒等变换。
这只验证识别链路，不能保证物理距离精度。实际跟随前需用实测距离验证内参；有明显误差时先做相机标定。
若之后加载了非零畸变参数，需用 image_proc 等对图像去畸变，再指定校正图像话题，不要继续直接把 raw 图像接给检测器。

脚本支持更换输入：

```bash
~/agx_ws/scripts/start_apriltag.sh 0.15 \
  image_topic:=/camera/color/image_rect \
  camera_info_topic:=/camera/color/camera_info
```

配置文件：config/apriltag_36h11_id0.yaml；启动文件：launch/apriltag_astra.launch.py。
这两个文件由脚本按工作空间路径直接读取，修改后无需 colcon build。

参考：https://github.com/christianrauch/apriltag_ros

## 画面边框和坐标轴可视化

相机与 AprilTag 检测启动后，另开终端运行：

```bash
~/agx_ws/scripts/start_apriltag_visualization.sh
```

在已有的 rqt_image_view 顶部下拉框选择 /apriltag/image_overlay。
若尚未打开查看器：

```bash
source ~/agx_ws/setup_humble.bash
ros2 run rqt_image_view rqt_image_view
```

黄色边框对应 ID 0 的四个角点；红 X、绿 Y、蓝 Z 是标记坐标轴，轴长默认 0.075 m。
顶部 camera x/y/z 是标记原点在相机光学坐标系中的位置，单位米。
无标记时显示 No ID 0；检测消息缺失时显示 Waiting for synchronized detections。
绘制使用与图像时间戳匹配的 TF；等待不到时只画边框并提示，不使用旧 TF。
此节点直接处理图像数据，避免本机 cv_bridge 与 NumPy 2 的 Python ABI 冲突。

三维查看坐标系：

```bash
source ~/agx_ws/setup_humble.bash
rviz2 -d ~/agx_ws/config/apriltag.rviz
```

RViz 固定坐标系为 camera_color_optical_frame，同时显示相机、target_tag 等 TF 和叠加图。
它们是相对相机的坐标，不代表相机与底盘的安装变换已标定。
RViz 的 TF 缓存可能保留已消失目标的最后位置；当前是否识别到目标，以新检测结果及叠加画面为准。
节点不发布 /cmd_vel。

可视化验证：已用独立测试话题验证黄色边框、匹配 TF 的红/绿轴，以及目标消失后清除标注。
实拍显示需要相机、检测节点、可视化节点同时运行。


## 可视化性能修复

图像发布使用 array('B')，避免 ROS Python 消息赋值逐字节检查。
渲染优先选择最新可用帧，发布后丢弃更早的缓存，避免旧画面倒退或积压。
检测/TF 等待上限分别缩短为约 120/50 ms。
detector.decimate 默认 2.0，降低四边形检测的计算量；远处很小的标记可能需要调回 1.0。
实测优化前叠加延迟中位数约 339 ms，优化后约 18 ms；帧率随相机和系统负载变化。

## 检测入口只保留最新帧

工作空间现在包含 apriltag_ros 3.4.0 源码的本地修改：
qos_profile=latest 使用 BEST_EFFORT、KEEP_LAST、depth=1；
max_image_age=0.1 在检测开始前跳过超过 100 ms 的旧图像。
当前正在计算的一帧会正常结束；下一次回调取等待队列中的最新帧，避免顺序追赶历史视频。
该策略会主动丢帧，不能提高相机自身采集帧率，也不能保证零延迟。
编译更新检测器：

```bash
cd ~/agx_ws
./scripts/build_humble.sh --packages-select apriltag_ros --allow-overriding apriltag_ros
```

启动仍使用 scripts/start_apriltag.sh 0.15，脚本会加载工作空间中的修改版。
系统 /opt/ros/humble 中的二进制包未修改。
