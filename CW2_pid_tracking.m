function CW2_pid_tracking()
%% CW2_PID_TRACKING
%  Closed-loop PID trajectory tracking for a differential-drive robot.
%  The robot tracks a straight-line diagonal path from (0,0) to (3,3).
%
%  PID Law:
%    u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*(de/dt)
%
%  Separate PID controllers are applied to X and Y errors,
%  producing linear (v) and angular (omega) velocity commands.

%% --- Time setup ---
dt      = 0.1;
t_total = 10;
t       = 0:dt:t_total;
N       = length(t);

%% --- Desired trajectory: diagonal straight line (0,0) → (3,3) ---
x_des = linspace(0, 3, N);
y_des = linspace(0, 3, N);

%% --- PID gains (tuned for smooth tracking) ---
Kp = 2.5;
Ki = 0.2;
Kd = 1.0;

%% --- Initial pose (facing 45° toward goal) ---
x     = 0;    y = 0;    theta = pi/4;
e_x_prev = 0; e_y_prev = 0;
int_ex   = 0; int_ey   = 0;

X = zeros(1,N);  Y = zeros(1,N);
Ex = zeros(1,N); Ey = zeros(1,N);

%% --- PID control loop ---
for i = 1:N
    % Position errors
    e_x = x_des(i) - x;
    e_y = y_des(i) - y;

    % Integral accumulation
    int_ex = int_ex + e_x * dt;
    int_ey = int_ey + e_y * dt;

    % Derivative terms
    d_ex = (e_x - e_x_prev) / dt;
    d_ey = (e_y - e_y_prev) / dt;

    % PID output → velocity commands
    v     = Kp*e_x + Ki*int_ex + Kd*d_ex;
    omega = Kp*e_y + Ki*int_ey + Kd*d_ey;

    % Euler integration
    x     = x + v * cos(theta) * dt;
    y     = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    % Store
    X(i) = x;   Y(i) = y;
    Ex(i) = e_x; Ey(i) = e_y;
    e_x_prev = e_x;
    e_y_prev = e_y;
end

%% --- Compute tracking error magnitude ---
err_mag = sqrt(Ex.^2 + Ey.^2);

%% --- Plot 1: Trajectory tracking ---
figure('Name','Sec 2 – PID Trajectory Tracking','NumberTitle','off');
plot(x_des, y_des, 'k--', 'LineWidth', 2, 'DisplayName', 'Desired Path');
hold on;
plot(X, Y, 'r-', 'LineWidth', 2, 'DisplayName', 'Actual Path (PID)');
plot(x_des(1), y_des(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor','g', 'DisplayName','Start');
plot(x_des(end), y_des(end), 'r*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName','Goal');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Section 2: PID Closed-Loop Trajectory Tracking');
legend('Location','northwest');

%% --- Plot 2: Error over time ---
figure('Name','Sec 2 – Tracking Error','NumberTitle','off');
subplot(3,1,1);
plot(t, Ex, 'b-','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('Error X (m)'); title('X Tracking Error');

subplot(3,1,2);
plot(t, Ey, 'r-','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('Error Y (m)'); title('Y Tracking Error');

subplot(3,1,3);
plot(t, err_mag, 'm-','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('|e| (m)'); title('Total Error Magnitude');
sgtitle('Section 2: PID Tracking Errors');

fprintf('   PID tracking complete. Final error: %.4f m\n', err_mag(end));
end