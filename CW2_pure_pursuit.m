function CW2_pure_pursuit()
%% CW2_PURE_PURSUIT
%  Waypoint-based path following using the Pure Pursuit controller.
%  Based on the practical session (test1.m).
%
%  Pure Pursuit geometry:
%    - A lookahead point is selected on the path at distance L_d ahead
%    - The controller steers toward that point using arc geometry
%    - Angular velocity: omega = (2*v * sin(alpha)) / L_d
%      where alpha is the heading error to the lookahead point
%
%  The robot model uses differentialDriveKinematics from
%  MATLAB Robotics System Toolbox.

%% --- Waypoints (manual path) ---
path = [2.00,  1.00;
        1.25,  1.75;
        5.25,  8.25;
        7.25,  8.75;
       11.75, 10.75;
       12.00, 10.00];

robotInitialLocation = path(1,:);
robotGoal            = path(end,:);
initialOrientation   = 0;                       % rad
robotCurrentPose     = [robotInitialLocation, initialOrientation]';

%% --- Robot kinematic model ---
robot = differentialDriveKinematics( ...
    'TrackWidth',    0.5, ...
    'VehicleInputs', 'VehicleSpeedHeadingRate');

%% --- Pure Pursuit controller ---
controller = controllerPurePursuit;
controller.Waypoints             = path;
controller.DesiredLinearVelocity = 0.6;    % m/s
controller.MaxAngularVelocity    = 2.0;    % rad/s
controller.LookaheadDistance     = 0.3;    % m

%% --- Simulation parameters ---
sampleTime     = 0.1;
goalRadius     = 0.1;                        % m
distanceToGoal = norm(robotInitialLocation - robotGoal);
vizRate        = rateControl(1/sampleTime);
frameSize      = robot.TrackWidth / 0.8;

%% --- Figure 1: desired path ---
figure('Name','Sec 5 – Desired Waypoints','NumberTitle','off');
plot(path(:,1), path(:,2), 'k-*', 'LineWidth', 1.5);
hold on;
plot(path(1,1),   path(1,2),   'go', 'MarkerSize',10,'MarkerFaceColor','g','DisplayName','Start');
plot(path(end,1), path(end,2), 'r*', 'MarkerSize',12,'LineWidth',2,        'DisplayName','Goal');
title('Section 5: Desired Waypoint Path');
xlabel('X (m)'); ylabel('Y (m)');
legend('Waypoints','Start','Goal','Location','best');
axis padded; grid on;

%% --- Figure 2: live simulation ---
figure('Name','Sec 5 – Pure Pursuit Tracking','NumberTitle','off');

step_count  = 0;
actual_path = robotCurrentPose(1:2)';

while distanceToGoal > goalRadius
    % Compute control commands from Pure Pursuit
    [v, omega] = controller(robotCurrentPose);

    % Propagate kinematics
    vel              = derivative(robot, robotCurrentPose, [v, omega]);
    robotCurrentPose = robotCurrentPose + vel * sampleTime;

    % Log actual path
    actual_path(end+1,:) = robotCurrentPose(1:2)'; %#ok<AGROW>

    % Update distance to goal
    distanceToGoal = norm(robotCurrentPose(1:2)' - robotGoal);
    step_count     = step_count + 1;

    % Visualise every step
    hold off;
    plot(path(:,1), path(:,2), 'k-*', 'LineWidth', 1.5, 'DisplayName','Waypoints');
    hold on;
    plot(actual_path(:,1), actual_path(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName','Actual Path');

    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot   = axang2quat([0, 0, 1, robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, ...
        'MeshFilePath', 'groundvehicle.stl', ...
        'Parent', gca, 'View', '2D', 'FrameSize', frameSize);

    xlim([0 14]); ylim([0 13]); grid on;
    title(sprintf('Section 5: Pure Pursuit  |  Step %d  |  Dist to Goal: %.2f m', ...
                   step_count, distanceToGoal));
    xlabel('X (m)'); ylabel('Y (m)');
    legend('Waypoints','Actual Path','Location','northwest');

    waitfor(vizRate);
end

fprintf('   Pure Pursuit complete. Steps: %d | Final dist: %.3f m\n', ...
    step_count, distanceToGoal);
end