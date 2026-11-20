# BUNKER 低速阶跃辨识

脚本：`scripts/bunker_identify.py`。只控制单个通道，另一个通道始终为零。默认预览，不运动。

## 准备

在空旷平整区域操作，遥控接管和急停可用。停止跟随、速度保护输出、键盘遥控和导航等所有 `/cmd_vel` 发布节点，只保留底盘驱动。不要同时启动总跟随流程。脚本发现其他发布者会中止，但 ROS 图发现有延迟，不应只依赖该检查。

```bash
cd /home/aixiaoxiao/agx_ws
source setup_humble.bash
ros2 topic info /cmd_vel --verbose
ros2 topic hz /bunker_status
```

## 首次检查

```bash
python3 scripts/bunker_identify.py --axis linear --amplitudes 0.05 --hold 2 --repeats 1
```

确认现场允许前进后，用相同参数加 `--execute`。即使速度很低，也可能因死区完全不动，这种数据仍有价值。反馈为零并不能证明没有运动。

```bash
python3 scripts/bunker_identify.py --axis linear --amplitudes 0.05 --hold 2 --repeats 1 --execute
```

观察运动方向和停止过程。Ctrl+C 会触发连续约一秒的零速度发送；通信中断、强制杀进程或断电时无法保证软件停车，应使用遥控或急停。

## 后续实验

先确认首轮数据和实际运动，再进行以下实验：

```bash
python3 scripts/bunker_identify.py --axis linear --execute
python3 scripts/bunker_identify.py --axis angular --execute
```

默认直线 0.05、0.10 m/s；默认转向 ±0.05、±0.10 rad/s；开头零速度固定保持两秒；各档保持三秒、间隔四秒，重复三轮。每次直线实验会累积前进距离，需要预留全程空间。固定间隔不代表已检测停稳，操作者应观察，必要时增大 `--rest`。

IMU 为可选的独立角速度观测，通过实际话题名传入：`--imu-topic /实际的IMU话题`。脚本不融合、不改变 IMU 坐标；后续分析需要确认安装方向、单位和时间戳。当前脚本不支持边走边转，先完成单通道实验。

## 输出

默认保存在当前目录 `data/system_identification/参数__时间戳/`：

- `commands.csv`：发布的线速度、角速度、实验阶段、单调时钟经过时间、ROS 发布时间。
- `feedback.csv`：底盘反馈速度、接收经过时间、ROS 接收时间、消息头时间。
- `imu.csv`：可选的 IMU z 轴角速度及时间，没有 IMU 时仅表头。
- `metadata.json`：参数、实验序列、完成或中止原因。

可用 `--output 路径` 指定一个尚不存在的目录。数据即时写出，异常时尽量保留已记录数据。CSV 指令是脚本发布值，不证明 CAN 实际接收；底盘反馈也不等于独立测得的真实车身运动。记录实验地面、载荷、电量、是否打滑等现场信息。

本脚本负责激励与记录，尚未自动拟合模型。应先检查曲线、死区、时间同步和左右差异，再选择一阶延迟或其他模型，并保留独立实验用于验证。

## 目录命名

默认目录名包含实际生效的通道、幅值序列及单位、保持时间、零速度间隔、重复次数、发布频率、反馈超时、是否记录 IMU，最后附时间戳。例如：

```text
linear_amp_0.05mps_initial2s_hold2s_rest4s_repeat1_rate30Hz_timeout0.5s_imuoff__20261105_213000_123456/
```

`angular` 为转向，`radps` 为 rad/s；负幅值保留负号，多个幅值以下划线分隔。预览也会显示预计保存路径，但不会创建目录。使用 `--output` 时保留用户指定目录名，完整参数仍保存于 `metadata.json`。已有实验目录不会自动改名。

开头的 baseline 零速度阶段固定为 2 秒，目录名使用 `initial2s` 标记；`--rest` 仅控制每档速度之后的零速度间隔。执行前另有约 3 秒的通信发现与反馈检查，此时尚未发布实验指令。

直线幅值允许绝对值不超过 0.3 m/s，转向幅值允许绝对值不超过 0.5 rad/s。以 0.3 m/s 保持 5 秒时，仅恒速段理想前进距离约 1.5 米，还需预留加减速和停车空间。

## 离线拟合一阶与二阶模型

无需启动 ROS 或连接小车，不发送任何指令：

```bash
cd /home/aixiaoxiao/agx_ws
python3 scripts/bunker_fit_response.py
```

依赖 numpy、scipy、matplotlib。可通过 `--data 数据根目录 --output 结果目录` 指定路径；结果目录中同名分析文件会覆盖，原始实验数据不修改。

默认结果位于 `data/identification_results/`，包括 `report.md`、`results.json`、`predictions.csv`、`linear_comparison.png`、`angular_comparison.png`。

一阶模型为 K exp(-Ls)/(tau s+1)，二阶为 K wn² exp(-Ls)/(s²+2 zeta wn s+wn²)，支持欠阻尼和过阻尼。使用实际指令变化时间、零阶保持输入、连续模型精确阶跃叠加；基线均值逐次扣除。每个幅值组最后一次完整实验保留为验证集，其余用于共同参数拟合。至少需要每档两次完整实验。仅当二阶验证 RMSE 降低超过 5% 时推荐增加阶次，这属于明确的工程选择规则，不是统计显著性检验。训练集 BIC 作为补充；反馈残差可能相关，BIC 的独立噪声假设不严格成立。

当前结果仅适用已测试方向、速度、载荷和地面；当前数据没有独立 IMU，不证明真实车身转向响应。延迟包含通信及软件调度。若参数接近搜索边界（results.json 中 near_bound），需要人工复核。参数不确定性与置信区间尚未计算，不能把小于采样间隔的延迟差异当作精确测量。当前验证仍来自同次采集环境，后续应在新实验上复验。
