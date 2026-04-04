function CW2_localisation()
%% CW2_LOCALISATION
%  Robot localisation combining dead-reckoning (odometry) with
%  noisy sensor readings using a Bayesian weighted fusion approach.
%
%  Key features (from practical sessions):
%    - Dead reckoning: integrates step movements without correction
%    - Sensor model:   GPS-like reading corrupted by Gaussian noise
%    - Moving average: smooths last 3 sensor readings
%    - Adaptive weights: shifts trust toward odometry when noise is high
%    - Bayesian fusion:  x_corrected = w1*odom + w2*sensor_avg
%
%  Task 2 extensions implemented:
%    Part 1 — Sensor noise increases over time (sensor degrades)
%    Part 2 — 3-step moving average filter on sensor readings
%    Part 4 — Adaptive weights based on noise level

%% --- Initial state ---
x = 0;  y = 0;  theta = 0;   % Corrected (fused) pose
odom_x = 0;  odom_y = 0;     % Pure dead-reckoning pose

step_size    = 1;
sensor_noise = 0.3;           % Base noise (will increase per step)
w1 = 0.3;  w2 = 0.7;         % Odometry / sensor weights

sensor_x_hist = [];  sensor_y_hist = [];   % Moving average buffers

% Path storage
x_odom_path = zeros(1,10);  y_odom_path = zeros(1,10);
x_corr_path = zeros(1,10);  y_corr_path = zeros(1,10);
noise_log   = zeros(1,10);
w1_log      = zeros(1,10);

%% --- Figure setup ---
figure('Name','Sec 3 – Robot Localisation','NumberTitle','off');
hold on; grid on;
axis([-5 12 -5 12]);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Section 3: Localisation — Dead Reckoning vs Bayesian Corrected');

%% --- Simulation loop (10 steps) ---
for i = 1:10

    % --- Part 1: Noise increases with each step ---
    sensor_noise = 0.3 + 0.1 * i;
    noise_log(i) = sensor_noise;

    % --- Random movement decision ---
    movement = randi([1 3]);

    % --- Dead reckoning update ---
    if movement == 1        % Move forward
        odom_x = odom_x + step_size * cosd(theta);
        odom_y = odom_y + step_size * sind(theta);
    elseif movement == 2    % Turn left
        theta = theta + 90;
    else                    % Turn right
        theta = theta - 90;
    end

    % --- Simulated sensor reading (noisy GPS) ---
    sensor_x = odom_x + normrnd(0, sensor_noise);
    sensor_y = odom_y + normrnd(0, sensor_noise);

    % --- Part 2: Moving average filter (window = 3) ---
    sensor_x_hist(end+1) = sensor_x; %#ok<AGROW>
    sensor_y_hist(end+1) = sensor_y; %#ok<AGROW>
    if length(sensor_x_hist) > 3
        sensor_x_hist(1) = [];
        sensor_y_hist(1) = [];
    end
    sensor_x_avg = mean(sensor_x_hist);
    sensor_y_avg = mean(sensor_y_hist);

    % --- Part 4: Adaptive weights based on noise level ---
    if sensor_noise > 1.0
        w1 = 0.7;  w2 = 0.3;   % High noise → trust odometry
    else
        w1 = 0.3;  w2 = 0.7;   % Low noise  → trust sensor
    end
    w1_log(i) = w1;

    % --- Bayesian weighted fusion ---
    x = w1 * odom_x + w2 * sensor_x_avg;
    y = w1 * odom_y + w2 * sensor_y_avg;

    % --- Store paths ---
    x_odom_path(i) = odom_x;  y_odom_path(i) = odom_y;
    x_corr_path(i) = x;       y_corr_path(i) = y;

    % --- Animate ---
    plot(odom_x, odom_y, 'r--o', 'MarkerSize', 7, 'MarkerFaceColor','r');
    plot(x, y,           'b-o',  'MarkerSize', 7, 'MarkerFaceColor','b');
    drawnow; pause(0.3);
end

legend('Dead Reckoning (odom)', 'Bayesian Corrected', 'Location','northwest');

% Compute total drift between odom and corrected path
drift = sqrt((x_odom_path - x_corr_path).^2 + (y_odom_path - y_corr_path).^2);

%% --- Plot 2: Sensor noise and adaptive weight evolution ---
figure('Name','Sec 3 – Noise & Weight Evolution','NumberTitle','off');
yyaxis left
plot(1:10, noise_log, 'r-o', 'LineWidth', 1.5);
ylabel('Sensor Noise \sigma');

yyaxis right
plot(1:10, w1_log, 'b--s', 'LineWidth', 1.5);
ylabel('Odometry Weight w_1');

xlabel('Step'); title('Section 3: Adaptive Noise and Weight Evolution');
legend('Sensor Noise', 'Odom Weight','Location','northwest'); grid on;

%% --- Plot 3: Drift between methods ---
figure('Name','Sec 3 – Localisation Drift','NumberTitle','off');
bar(1:10, drift, 'FaceColor',[0.2 0.6 0.8]);
xlabel('Step'); ylabel('Drift (m)');
title('Section 3: Positional Drift — Odom vs Bayesian Corrected');
grid on;

fprintf('   Localisation complete. Max drift: %.3f m\n', max(drift));
end