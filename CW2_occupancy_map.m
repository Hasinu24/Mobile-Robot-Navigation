function CW2_occupancy_map()
%% CW2_OCCUPANCY_MAP
%  Demonstrates 2D binary occupancy map construction and robot
%  straight-line motion simulation using differential-drive kinematics.
%
%  Based on formative assessment Part 4 (practical.m).
%  Extended with:
%    - Collision detection against occupied cells
%    - Path length computation
%    - Cleaner obstacle layout representing indoor furniture/walls

%% --- Step 1: Build occupancy map ---
% 10 x 10 metre map, 10 cells per metre (100x100 grid cells)
map = binaryOccupancyMap(10, 10, 10);

% Obstacle positions (m) — represent walls and furniture
obstaclePositions = [
    4.0, 1.0;  4.0, 1.5;  4.0, 2.0;
    4.5, 1.0;  4.5, 1.5;  4.5, 2.0;
    5.0, 1.0;  5.0, 1.5;  5.0, 2.0;
    % Additional wall segment
    3.0, 3.5;  3.5, 3.5;  4.0, 3.5;
    6.0, 2.5;  6.5, 2.5;  7.0, 2.5;
];
setOccupancy(map, obstaclePositions, 1);

%% --- Step 2: Robot initial pose ---
x     = 0.5;   % m
y     = 1.0;   % m
theta = 0;     % rad — facing right (East)

%% --- Step 3: Motion parameters ---
v     = 0.15;   % Linear velocity (m/s)
omega = 0;      % Straight line (no rotation)
dt    = 0.1;    % Time step (s)
steps = 120;    % Total simulation steps

% Storage
X_path = zeros(1, steps);
Y_path = zeros(1, steps);
collision_flag = false;

%% --- Step 4: Display initial map ---
figure('Name','Sec 4 – Occupancy Map Navigation','NumberTitle','off');
show(map); hold on;
title('2D Occupancy Map — Straight Line Motion (A to B)');
xlabel('X (m)'); ylabel('Y (m)');

% Mark start
plot(x, y, 'go', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName','Start A');

%% --- Step 5: Simulate motion with collision checking ---
for i = 1:steps
    % Differential drive kinematics update
    x_new = x + v * cos(theta) * dt;
    y_new = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    % Collision check — is new position occupied?
    if checkOccupancy(map, [x_new, y_new])
        warning('Collision detected at step %d! Halting.', i);
        collision_flag = true;
        X_path(i) = x;
        Y_path(i) = y;
        break;
    end

    x = x_new;  y = y_new;
    X_path(i) = x;
    Y_path(i) = y;

    % Animate robot position
    plot(x, y, 'b.', 'MarkerSize', 8);
    drawnow limitrate;
    pause(0.02);
end

%% --- Step 6: Mark goal and finalise ---
plot(x, y, 'r^', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName','Goal B');

h1 = plot(nan, nan, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName','Start: A');
h2 = plot(nan, nan, 'b.', 'MarkerSize', 10,              'DisplayName','Robot Path');
h3 = plot(nan, nan, 'r^', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName','Goal: B');
legend([h1,h2,h3], 'Location','northeast');
grid on;

%% --- Compute path length ---
valid = X_path ~= 0 | Y_path ~= 0;
px = X_path(valid);  py = Y_path(valid);
if length(px) > 1
    seg_lengths  = sqrt(diff(px).^2 + diff(py).^2);
    total_length = sum(seg_lengths);
else
    total_length = 0;
end

if collision_flag
    fprintf('   Occupancy map: collision detected. Path length: %.2f m\n', total_length);
else
    fprintf('   Occupancy map: goal reached. Path length: %.2f m\n', total_length);
end
end