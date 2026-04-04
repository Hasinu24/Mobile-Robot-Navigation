function CW2_forward_kinematics()
%% CW2_FORWARD_KINEMATICS
%  Demonstrates differential-drive forward kinematics using a unicycle model.
%  The robot follows a figure-eight open-loop trajectory produced by
%  sinusoidally varying wheel speeds.
%
%  Robot Model:
%    Linear velocity:  v = (r/2) * (w_r + w_l)
%    Angular velocity: omega = (r/d) * (w_r - w_l)
%
%  Pose update (Euler integration):
%    x(t+dt)     = x(t) + v*cos(theta)*dt
%    y(t+dt)     = y(t) + v*sin(theta)*dt
%    theta(t+dt) = theta(t) + omega*dt

%% --- Robot physical parameters ---
r = 0.05;   % Wheel radius (m)
d = 0.2;    % Wheelbase (m)

%% --- Time setup ---
dt      = 0.1;    % Time step (s)
t_total = 10;     % Total simulation time (s)
t       = 0:dt:t_total;
N       = length(t);

%% --- Wheel speed profile (sinusoidal → figure-eight path) ---
% Left and right wheel angular velocities (rad/s)
w_l = 1 + 0.5 * sin(0.5 * t);
w_r = 1 - 0.5 * sin(0.5 * t);

%% --- Initialise state ---
x     = 0; y = 0; theta = 0;
X     = zeros(1,N);
Y     = zeros(1,N);
Theta = zeros(1,N);

%% --- Forward kinematics simulation loop ---
for i = 1:N
    % Compute body-frame velocities from wheel speeds
    v     = (r/2) * (w_r(i) + w_l(i));
    omega = (r/d) * (w_r(i) - w_l(i));

    % Euler integration to update world-frame pose
    x     = x + v * cos(theta) * dt;
    y     = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    % Record
    X(i) = x;  Y(i) = y;  Theta(i) = theta;
end

%% --- Velocity profiles ---
v_linear  = (r/2) .* (w_r + w_l);
v_angular = (r/d) .* (w_r - w_l);

%% --- Plot 1: Trajectory ---
figure('Name','Sec 1 – Forward Kinematics Trajectory','NumberTitle','off');
plot(X, Y, 'b-', 'LineWidth', 2); hold on;
plot(X(1),Y(1),'go','MarkerSize',12,'MarkerFaceColor','g','DisplayName','Start');
plot(X(end),Y(end),'r*','MarkerSize',12,'LineWidth',2,'DisplayName','End');
grid on; axis equal;
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Section 1: Open-Loop Trajectory ');
legend('Trajectory','Start','End','Location','best');

%% --- Plot 2: Heading angle over time ---
figure('Name','Sec 1 – Heading over Time','NumberTitle','off');
plot(t, rad2deg(Theta), 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Heading \theta (deg)');
title('Section 1: Robot Heading vs Time');

%% --- Plot 3: Velocity profiles ---
figure('Name','Sec 1 – Velocity Profiles','NumberTitle','off');
subplot(2,1,1);
plot(t, v_linear, 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('v (m/s)');
title('Linear Velocity');

subplot(2,1,2);
plot(t, v_angular, 'r-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('\omega (rad/s)');
title('Angular Velocity');
sgtitle('Section 1: Velocity Profiles');

fprintf('   Forward kinematics complete.\n');
end