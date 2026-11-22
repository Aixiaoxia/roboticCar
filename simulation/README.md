# Simulink 跟随控制器

`speed.slx` 和 `parameters.m` 是原始仿真文件。在 MATLAB 将当前文件夹切换到本目录后执行：

```matlab
run('parameters.m')
open_system('speed')
```

模型使用距离、偏角外环和线速度、角速度内环。四个控制块的 D 增益都为零。底盘传递函数、纯延迟和虚拟相机用于仿真。ROS 从 AprilTag 获取距离和偏角，串级模式从 `/bunker_status` 获取底盘速度。

## 切换模式

编辑仓库根目录的 `follow_config.yaml`，在 `tag_follower.ros__parameters` 下设置 `controller_mode`，重启跟随节点后生效。

| 值 | 行为 |
| --- | --- |
| `proportional` | 原有比例控制，默认选择。 |
| `outer_pi` | 距离、偏角 PI 输出候选速度。 |
| `cascade_pi` | 距离、偏角 PI 输出速度参考，速度 PI 根据 `/bunker_status` 校正候选速度。反馈无效、缺失或超时则输出零速并清空积分。 |

| 通道 | Kp | Ki |
| --- | ---: | ---: |
| 距离外环 | 0.6 | 0.12 |
| 偏角外环 | 2.0 | 0.02 |
| 线速度内环 | 0.5 | 3.0 |
| 角速度内环 | 0.4 | 4.0 |

这些增益来自仿真，可通过同一配置文件调整。ROS 使用原有平面距离、距离死区及目标距离配置，每个控制周期按经过的时间积分，默认控制频率为 20 Hz。车体坐标下，目标在左侧时偏角为正。跟随保持、目标丢失和无效观测会清空积分。原地对准时保持线速度为零。候选速度仍经过独立 `velocity_guard` 的人工使能、路径互锁、限幅与加速度限制。

## 预览和对比

在已有相机和 AprilTag 检测的 ROS 2 工作区内，先用 `outer_pi` 做预览：

```bash
./scripts/build_humble.sh --packages-select bunker_tag_follower
source ./setup_humble.bash
./scripts/start_tag_follow.sh hardware:=false
```

在另一个已加载环境的终端查看 `/follow/state` 和 `/follow/cmd_vel_preview`。比较目标左移、右移、接近和丢失时的输出。`/follow/state` 包含控制模式、外环速度参考和候选指令。验证串级模式时还需提供 `/bunker_status`，检查反馈超时后速度是否归零。增益需结合实际跟随距离、视觉延迟、底盘响应和速度限制重新整定。实车启动条件见 [完整使用指南](../docs/Bunker_AprilTag_ROS2_完整使用指南.md)。
