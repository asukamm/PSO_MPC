%% run_comparison.m
%% ============================================================
%% NMPC vs PID vs LQR 对比实验
%% 差速轮式机器人轨迹跟踪
%% 三组工况: 直线(低速) / 圆形(中速) / S形(高速)
%% 三种控制器: NMPC / PID / LQR
%% ============================================================

function run_comparison()
close all; clc;
fprintf('\n');
fprintf('============================================================\n');
fprintf('  差速轮式机器人控制器对比实验\n');
fprintf('  NMPC vs PID vs LQR\n');
fprintf('============================================================\n\n');

%% 1. 环境初始化
fprintf('[1/6] 初始化环境...\n');
addpath('../');
addpath('../controllers');
addpath('../models');
addpath('../params');
addpath('../simulation');
addpath('../visualization');

params = nova_carter_params();
dt = params.dt;
model = differential_drive_model();

T_sim = 30.0;
N_steps = round(T_sim / dt);
fprintf('     仿真时长: %.1f s, 步长: %.3f s, 总步数: %d\n', T_sim, dt, N_steps);

%% 2. 生成三组参考轨迹
fprintf('[2/6] 生成参考轨迹...\n');

v1 = 0.5;
x_ref_set1 = gen_straight_line(N_steps, 30, dt, v1);
fprintf('     Set1 直线轨迹: v=%.1f m/s\n', v1);

v2 = 1.0;
R2 = 5.0;
x_ref_set2 = gen_circle(N_steps, 30, dt, R2, v2);
fprintf('     Set2 圆形轨迹: v=%.1f m/s, R=%.1f m\n', v2, R2);

v3 = 1.2;
x_ref_set3 = gen_s_curve(N_steps, 30, dt, v3);
fprintf('     Set3 S形曲线: v=%.1f m/s\n', v3);

%% 3. 定义控制器参数
fprintf('[3/6] 配置控制器...\n');

% NMPC参数
N_mpc = 20;
Q_mpc = diag([40, 40, 10]);
R_mpc = diag([0.1, 0.5]);
S_mpc = diag([1.0, 2.0]);
Qf_mpc = 50 * Q_mpc;

% PID参数
PID_params.v_Kp = 3.0;
PID_params.v_Ki = 0.1;
PID_params.v_Kd = 0.2;
PID_params.w_Kp = 4.0;
PID_params.w_Ki = 0.05;
PID_params.w_Kd = 0.3;

% LQR参数
Q_lqr = diag([10, 10, 1]);
R_lqr = diag([0.1, 0.5]);

%% 4. 运行所有仿真 (3x3=9组)
fprintf('[4/6] 运行仿真(共9组)...\n');

trajectories = {x_ref_set1, x_ref_set2, x_ref_set3};
traj_names = {'Set1_Straight', 'Set2_Circle', 'Set3_SCurve'};
traj_labels = {'直线(低速)', '圆形(中速)', 'S形(高速)'};
controllers = {'NMPC', 'PID', 'LQR'};

results = struct();
N_trajs = 3;

for ti = 1:N_trajs
    x_ref = trajectories{ti};
    traj_name = traj_names{ti};

    for ci = 1:3
        ctrl_name = controllers{ci};
        fprintf('     运行 %s | %s ...\n', traj_name, ctrl_name);

        [x_hist, u_hist, solve_times] = run_simulation(...
            ctrl_name, x_ref, params, model, dt, N_steps, ...
            N_mpc, Q_mpc, R_mpc, S_mpc, Qf_mpc, ...
            PID_params, Q_lqr, R_lqr);

        [rmse_pos, rmse_heading, max_pos_err] = compute_errors(x_hist, x_ref, dt);

        results(ti, ci).name = [traj_name '_' ctrl_name];
        results(ti, ci).x_hist = x_hist;
        results(ti, ci).u_hist = u_hist;
        results(ti, ci).rmse_pos = rmse_pos;
        results(ti, ci).rmse_heading = rmse_heading;
        results(ti, ci).max_pos_err = max_pos_err;
        results(ti, ci).solve_times = solve_times;

        fprintf('        RMSE位置: %.3f m, RMSE航向: %.2f deg\n', ...
            rmse_pos, rad2deg(rmse_heading));
    end
end

%% 5. 绘制对比图
fprintf('[5/6] 绘制对比图...\n');

plot_trajectory_comparison(results, trajectories, traj_labels, dt, N_steps);
plot_rmse_bars(results, traj_labels, controllers);
plot_error_details(results, trajectories, 2, traj_labels{2}, dt, N_steps);

%% 6. 输出汇总表
fprintf('[6/6] 输出汇总结果...\n\n');
fprintf('============================================================\n');
fprintf('  结果汇总\n');
fprintf('============================================================\n');
fprintf('  %-16s | %-8s | %-8s | %-8s | %-8s\n', '工况', '指标', 'NMPC', 'PID', 'LQR');
fprintf('  %s\n', repmat('-', 1, 55));

for ti = 1:N_trajs
    fprintf('  %-16s | %-8s | %-8.4f | %-8.4f | %-8.4f\n', ...
        traj_labels{ti}, 'RMSE(m)', ...
        results(ti,1).rmse_pos, results(ti,2).rmse_pos, results(ti,3).rmse_pos);
    fprintf('  %-16s | %-8s | %-8.2f | %-8.2f | %-8.2f\n', ...
        '', '航向(deg)', ...
        rad2deg(results(ti,1).rmse_heading), ...
        rad2deg(results(ti,2).rmse_heading), ...
        rad2deg(results(ti,3).rmse_heading));
    fprintf('  %-16s | %-8s | %-8.4f | %-8.4f | %-8.4f\n', ...
        '', '最大误差(m)', ...
        results(ti,1).max_pos_err, results(ti,2).max_pos_err, results(ti,3).max_pos_err);
    if ti < N_trajs
        fprintf('  %s\n', repmat('-', 1, 55));
    end
end
fprintf('============================================================\n\n');
fprintf('对比实验完成！共生成3张对比图。\n');
end % run_comparison


%% ============================================================
%% 子函数: 运行单次仿真
%% ============================================================
function [x_hist, u_hist, solve_times] = run_simulation(...
    ctrl_type, x_ref, params, model, dt, N_steps, ...
    N_mpc, Q_mpc, R_mpc, S_mpc, Qf_mpc, ...
    PID_params, Q_lqr, R_lqr)

x_current = x_ref(:, 1) + [0; 0.3; 0.05];
u_last = [0; 0];

x_hist = zeros(3, N_steps + 1);
u_hist = zeros(2, N_steps);
x_hist(:, 1) = x_current;
solve_times = zeros(1, N_steps);

switch ctrl_type
    case 'NMPC'
        nmpc = nmpc_controller(N_mpc, Q_mpc, R_mpc, S_mpc, Qf_mpc);
    case 'PID'
        int_err_x = 0; int_err_y = 0;
        prev_err_x = 0; prev_err_y = 0;
end

for k = 1:N_steps
    switch ctrl_type
        case 'NMPC'
            x_ref_seg = x_ref(:, k:min(k+N_mpc, size(x_ref, 2)));
            tic;
            [u_opt, ~, ~] = nmpc.solve(x_current, x_ref_seg, u_last);
            solve_times(k) = toc;
            u_cmd = u_opt;

        case 'PID'
            tic;
            ref_k = x_ref(:, k);
            e_x = ref_k(1) - x_current(1);
            e_y = ref_k(2) - x_current(2);
            theta = x_current(3);
            e_x_body = cos(theta) * e_x + sin(theta) * e_y;
            e_y_body = -sin(theta) * e_x + cos(theta) * e_y;

            if k > 1
                v_ref = norm(x_ref(:, k) - x_ref(:, k-1)) / dt;
            else
                v_ref = 0.5;
            end

            int_err_x = int_err_x + e_x_body * dt;
            deriv_err_x = (e_x_body - prev_err_x) / dt;
            v_cmd = v_ref + PID_params.v_Kp * e_x_body ...
                + PID_params.v_Ki * int_err_x ...
                + PID_params.v_Kd * deriv_err_x;
            v_cmd = max(0, min(3.0, v_cmd));

            int_err_y = int_err_y + e_y_body * dt;
            deriv_err_y = (e_y_body - prev_err_y) / dt;
            w_cmd = PID_params.w_Kp * e_y_body ...
                + PID_params.w_Ki * int_err_y ...
                + PID_params.w_Kd * deriv_err_y;
            w_cmd = max(-pi/2, min(pi/2, w_cmd));

            prev_err_x = e_x_body;
            prev_err_y = e_y_body;
            u_cmd = [v_cmd; w_cmd];
            solve_times(k) = toc;

        case 'LQR'
            tic;
            ref_k = x_ref(:, k);
            e_state = x_current - ref_k;
            e_state(3) = myWrapToPi(e_state(3));

            theta_r = ref_k(3);
            v_r = 0.5;

            A_c = [0, 0, -v_r * sin(theta_r);
                   0, 0,  v_r * cos(theta_r);
                   0, 0,  0];
            B_c = [cos(theta_r), 0;
                   sin(theta_r), 0;
                   0,            1];

            A_d = eye(3) + A_c * dt;
            B_d = B_c * dt;

            K_lqr = solve_dare(A_d, B_d, Q_lqr, R_lqr);

            u_ref = [v_r; 0];
            u_cmd = u_ref - K_lqr * e_state;
            u_cmd = max([0; -pi/2], min([3.0; pi/2], u_cmd));
            solve_times(k) = toc;
    end

    x_current = model.dynamics_discrete(x_current, u_cmd);
    x_hist(:, k+1) = x_current;
    u_hist(:, k) = u_cmd;
    u_last = u_cmd;
end
end


%% ============================================================
%% 子函数: 误差计算
%% ============================================================
function [rmse_pos, rmse_heading, max_pos_err] = compute_errors(x_hist, x_ref, dt)
start_idx = round(2.0 / dt) + 1;
N = min(size(x_hist, 2), size(x_ref, 2));
if start_idx > N, start_idx = 1; end

pos_err = sqrt((x_hist(1, start_idx:N) - x_ref(1, start_idx:N)).^2 + ...
               (x_hist(2, start_idx:N) - x_ref(2, start_idx:N)).^2);
rmse_pos = sqrt(mean(pos_err.^2));
max_pos_err = max(pos_err);

heading_err = x_hist(3, start_idx:N) - x_ref(3, start_idx:N);
heading_err = myWrapToPi(heading_err);
rmse_heading = sqrt(mean(heading_err.^2));
end


%% ============================================================
%% 子函数: 轨迹对比图
%% ============================================================
function plot_trajectory_comparison(results, trajectories, traj_labels, dt, N_steps)
figure('Name', '控制器轨迹对比', 'Position', [50, 50, 1400, 1000]);
controllers = {'NMPC', 'PID', 'LQR'};
colors = {'b-', 'r-', 'g-'};

for ti = 1:3
    for ci = 1:3
        subplot(3, 3, (ti-1)*3 + ci);
        x_ref = trajectories{ti};
        N_plot = min(size(x_ref, 2), N_steps + 1);
        plot(x_ref(1, 1:N_plot), x_ref(2, 1:N_plot), 'k--', 'LineWidth', 1.5); hold on;

        x_hist = results(ti, ci).x_hist;
        N_act = min(size(x_hist, 2), N_plot);
        plot(x_hist(1, 1:N_act), x_hist(2, 1:N_act), colors{ci}, 'LineWidth', 2);
        plot(x_hist(1, 1), x_hist(2, 1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
        plot(x_hist(1, N_act), x_hist(2, N_act), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

        grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)');
        title([controllers{ci} ' | ' traj_labels{ti}], 'FontWeight', 'bold');
        if ci == 3
            legend({'参考', '实际', '起点', '终点'}, 'Location', 'best');
        end
    end
end
sgtitle('差速轮式机器人轨迹跟踪对比: NMPC vs PID vs LQR', 'FontSize', 14, 'FontWeight', 'bold');
end


%% ============================================================
%% 子函数: RMSE柱状图
%% ============================================================
function plot_rmse_bars(results, traj_labels, controllers)
figure('Name', 'RMSE对比', 'Position', [100, 100, 800, 600]);

subplot(2, 1, 1);
rmse_data = zeros(3, 3);
for ti = 1:3
    for ci = 1:3
        rmse_data(ti, ci) = results(ti, ci).rmse_pos;
    end
end
bar(rmse_data);
set(gca, 'XTickLabel', traj_labels);
xlabel('工况'); ylabel('位置RMSE (m)');
title('位置跟踪误差对比 (RMSE)');
legend(controllers, 'Location', 'best'); grid on;

subplot(2, 1, 2);
heading_data = zeros(3, 3);
for ti = 1:3
    for ci = 1:3
        heading_data(ti, ci) = rad2deg(results(ti, ci).rmse_heading);
    end
end
bar(heading_data);
set(gca, 'XTickLabel', traj_labels);
xlabel('工况'); ylabel('航向RMSE (deg)');
title('航向跟踪误差对比 (RMSE)');
legend(controllers, 'Location', 'best'); grid on;
sgtitle('控制器性能对比', 'FontSize', 14, 'FontWeight', 'bold');
end


%% ============================================================
%% 子函数: 误差细节
%% ============================================================
function plot_error_details(results, trajectories, traj_idx, traj_name, dt, N_steps)
figure('Name', ['误差详情 - ' traj_name], 'Position', [150, 150, 1000, 800]);
controllers = {'NMPC', 'PID', 'LQR'};
colors = {'b', 'r', 'g'};
t = (0:N_steps) * dt;

for ci = 1:3
    x_hist = results(traj_idx, ci).x_hist;
    x_ref = trajectories{traj_idx};
   N_p = min([size(x_hist, 2), size(x_ref, 2), N_steps + 1]);

    subplot(3, 1, 1);
    e_x = x_hist(1, 1:N_p) - x_ref(1, 1:N_p);
    plot(t(1:N_p), e_x, colors{ci}, 'LineWidth', 1.5); hold on;

    subplot(3, 1, 2);
    e_y = x_hist(2, 1:N_p) - x_ref(2, 1:N_p);
    plot(t(1:N_p), e_y, colors{ci}, 'LineWidth', 1.5); hold on;

    subplot(3, 1, 3);
    e_th = x_hist(3, 1:N_p) - x_ref(3, 1:N_p);
    plot(t(1:N_p), rad2deg(myWrapToPi(e_th)), colors{ci}, 'LineWidth', 1.5); hold on;
end

subplot(3, 1, 1); ylabel('e_x (m)'); title([traj_name ' - X方向误差']); grid on; legend(controllers);
subplot(3, 1, 2); ylabel('e_y (m)'); title([traj_name ' - Y方向误差']); grid on; legend(controllers);
subplot(3, 1, 3); xlabel('时间 (s)'); ylabel('e_theta (deg)'); title([traj_name ' - 航向误差']); grid on; legend(controllers);
sgtitle(['跟踪误差详情: ' traj_name], 'FontSize', 14, 'FontWeight', 'bold');
end


%% ============================================================
%% 轨迹生成函数
%% ============================================================
function x_ref = gen_straight_line(N_steps, N_horizon, dt, v)
N_total = N_steps + N_horizon + 1;
x_ref = zeros(3, N_total);
for k = 1:N_total
    t = (k-1) * dt;
    x_ref(:, k) = [v * t; 0; 0];
end
end

function x_ref = gen_circle(N_steps, N_horizon, dt, R, v)
N_total = N_steps + N_horizon + 1;
omega = v / R;
x_ref = zeros(3, N_total);
for k = 1:N_total
    t = (k-1) * dt;
    theta = omega * t;
    x = R * cos(theta);
    y = R * sin(theta);
    heading = myWrapToPi(theta + pi/2);
    x_ref(:, k) = [x; y; heading];
end
end

function x_ref = gen_s_curve(N_steps, N_horizon, dt, v)
N_total = N_steps + N_horizon + 1;
x_ref = zeros(3, N_total);
A = 4.0; freq = 0.15;
for k = 1:N_total
    t = (k-1) * dt;
    x = v * t;
    y = A * sin(freq * x);
    heading = atan2(A * freq * cos(freq * x), 1);
    x_ref(:, k) = [x; y; heading];
end
end


%% ============================================================
%% DARE求解器
%% ============================================================
function K = solve_dare(A, B, Q, R)
P = Q;
for i = 1:1000
    P_next = A' * P * A - A' * P * B / (R + B' * P * B) * B' * P * A + Q;
    if norm(P_next - P, 'fro') < 1e-10
        break;
    end
    P = P_next;
end
K = (R + B' * P * B) \ (B' * P * A);
end


%% ============================================================
%% wrapToPi
%% ============================================================
function wrapped = myWrapToPi(angle)
wrapped = angle - 2*pi * floor((angle + pi) / (2*pi));
end
