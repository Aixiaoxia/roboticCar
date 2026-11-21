
%% ============================================================
% parameters.m
% 二维码移动目标跟随仿真参数
% 绝对世界坐标系 + 小车运动学 + 二阶动力学
%% ============================================================

%% 1. 仿真采样时间
Ts = 0.01;                   % 视觉更新周期 (s)

%% 2. 小车初始状态
robot_x0 = 0.0;             % 小车世界坐标 X (m)
robot_y0 = 0.0;             % 小车世界坐标 Y (m)
robot_psi0 = 0.0;           % 小车初始航向角 (rad)

robot_plane_z = 0.0;        % 小车运动平面高度 (m)

%% 3. 摄像头安装位置
camera_forward_offset = 0.0; % 相对小车中心前移 (m)
camera_right_offset = 0.0;   % 相对小车中心右移 (m)

camera_z0 = robot_plane_z;   % 摄像头高度 (m)

%% 4. 二维码初始位置
qr_x0 = 10.0;                % 世界坐标 X (m)
qr_y0 = 0.5;                % 世界坐标 Y (m)

plane_gap = 0.3;            % 两运动平面高度差 (m)
qr_z0 = robot_plane_z + plane_gap;

%% 5. 二维码运动速度（世界坐标系）
qr_vx = 0.10;               % X方向速度 (m/s)
qr_vy = 0.00;               % Y方向速度 (m/s)

%% 6. 外环控制目标
target_position = 3.0;      % 期望三维距离 (m)
target_angle = 0.0;         % 期望偏转角 (rad)

%% 7. 组合成 Simulink 参数向量
% MATLAB Function 第三个输入端口 p 使用该向量

p0 = [ ...
    Ts, ...
    robot_x0, ...
    robot_y0, ...
    robot_psi0, ...
    qr_x0, ...
    qr_y0, ...
    camera_z0, ...
    qr_z0, ...
    qr_vx, ...
    qr_vy, ...
    camera_forward_offset, ...
    camera_right_offset ...
    ];

%% 8. 参数检查
fprintf('\n========== 仿真参数 ==========\n');
fprintf('采样时间：%.2f s\n',Ts);
fprintf('小车初始位置：(%.2f, %.2f)\n',robot_x0,robot_y0);
fprintf('二维码初始位置：(%.2f, %.2f, %.2f)\n',...
    qr_x0,qr_y0,qr_z0);
fprintf('二维码速度：(%.2f, %.2f) m/s\n',qr_vx,qr_vy);
fprintf('平面高度差：%.2f m\n',plane_gap);
fprintf('期望跟随距离：%.2f m\n',target_position);
fprintf('==============================\n');
