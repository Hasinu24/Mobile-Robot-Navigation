function results = CW2_navigation_comparison()
clc;

%% --- Build the three maps ---
maps = cell(1,3);
for k = 1:3
    maps{k} = CW2_build_map(k);
end

%% --- Safe start and goal ---
start_xy = [3, 3];
goal_xy  = [22, 22];

%% --- Run all experiments ---
results = cell(1,6);
algos   = {'astar','astar','astar','potentialfield','potentialfield','potentialfield'};
map_ids = [1, 2, 3, 1, 2, 3];

for i = 1:6
    fprintf('Running %s on Map %d...\n', algos{i}, map_ids(i));
    results{i} = CW2_run_experiment(maps{map_ids(i)}, algos{i}, map_ids(i), start_xy, goal_xy);
end

%% --- Show comparison charts ---
CW2_plot_comparison(results);

disp('All done! Check the 6 maps + bar charts.');
end

%% ====================================================================
function map = CW2_build_map(map_id)
    sz = 25; res = 1;
    g = zeros(sz, sz);

    % Border walls
    g(1,:) = 1; g(end,:) = 1; g(:,1) = 1; g(:,end) = 1;

    switch map_id
        case 1 % Sparse room
            g(8:12, 8:10) = 1;
            g(15:18, 6:8) = 1;
            g(5:8, 15:18) = 1;
            g(16:20, 16:19) = 1;

        case 2 % Maze corridors
            g(7, 3:16) = 1;   g(7, 18:23) = 1;
            g(13, 3:8) = 1;   g(13,11:23) = 1;
            g(19, 3:15) = 1;  g(19,17:23) = 1;
            g(3:7, 12) = 1;
            g(13:19,17) = 1;
            g(7:13, 6) = 1;

        case 3 % Cluttered warehouse
            for row = [5, 9, 13, 17, 21]
                g(row, 4:10) = 1;
                g(row, 14:21) = 1;
            end
            px = [6, 11, 16, 20];
            py = [4, 8, 12, 16, 20];
            for xi = px
                for yi = py
                    if xi < sz-1 && yi < sz-1
                        g(yi:yi+1, xi:xi+1) = 1;
                    end
                end
            end
    end

    map = binaryOccupancyMap(g, res);
end

%% ====================================================================
function result = CW2_run_experiment(map, algorithm, map_id, start_xy, goal_xy)
    result.algorithm    = algorithm;
    result.map_id       = map_id;
    result.success      = false;
    result.path         = [];
    result.path_length  = Inf;
    result.time_elapsed = Inf;

    tic;
    switch lower(algorithm)
        case 'astar'
            [path, ok] = CW2_astar(map, start_xy, goal_xy, map_id);
        case 'potentialfield'
            [path, ok] = CW2_apf(map, start_xy, goal_xy, map_id);
        otherwise
            error('Unknown algorithm: %s', algorithm);
    end
    result.time_elapsed = toc;
    result.success = ok;

    if ok && ~isempty(path)
        result.path = path;
        d = diff(path,1,1);
        result.path_length = sum(sqrt(sum(d.^2,2)));
    end
end

%% ====================================================================
function [path, success] = CW2_astar(map, start_xy, goal_xy, map_id)
    success = false;
    path = [];

    map_inf = copy(map);
    inflate(map_inf, 0.5);

    start_xy = CW2_find_free_point(map_inf, start_xy);
    goal_xy  = CW2_find_free_point(map_inf, goal_xy);

    if checkOccupancy(map_inf, start_xy)
        disp('A* failed - start is occupied after inflation');
        return;
    end
    if checkOccupancy(map_inf, goal_xy)
        disp('A* failed - goal is occupied after inflation');
        return;
    end

    planner = plannerAStarGrid(map_inf);

    gridStart = world2grid(map_inf, start_xy);
    gridGoal  = world2grid(map_inf, goal_xy);

    path_grid = plan(planner, gridStart, gridGoal);

    if isempty(path_grid)
        disp('A* failed - no path found');
        return;
    end

    if isprop(path_grid, 'Locations')
        path = path_grid.Locations;
    elseif isnumeric(path_grid)
        path = grid2world(map_inf, path_grid);
    else
        try
            path = grid2world(map_inf, path_grid.States);
        catch
            error('Unsupported path output from plannerAStarGrid.');
        end
    end

    success = true;

    figure('Name',sprintf('A* Map %d',map_id),'NumberTitle','off');
    show(map); hold on;
    title(sprintf('A* Navigation — Map %d',map_id));
    xlabel('X (m)'); ylabel('Y (m)');
    plot(path(:,1), path(:,2), 'b-', 'LineWidth',2);
    plot(start_xy(1),start_xy(2),'go','MarkerSize',12,'MarkerFaceColor','g');
    plot(goal_xy(1), goal_xy(2),'r*','MarkerSize',14,'LineWidth',2);

    rh = plot(start_xy(1),start_xy(2),'ko','MarkerSize',10,'MarkerFaceColor',[0.2 0.6 1]);
    for k = 1:3:size(path,1)
        set(rh,'XData',path(k,1),'YData',path(k,2));
        drawnow limitrate;
        pause(0.02);
    end
    set(rh,'XData',path(end,1),'YData',path(end,2));
    drawnow;
end

%% ====================================================================
function [path, success] = CW2_apf(map, start_xy, goal_xy, map_id)
    success = false;
    path = start_xy;

    k_att = 1.0;
    k_rep = 150.0;
    d0 = 3.0;
    step = 0.2;
    max_it = 3000;
    tol = 0.5;

    current = start_xy(:)';
    goal = goal_xy(:)';

    [rows, cols] = find(occupancyMatrix(map) > 0.5);
    if isempty(rows)
        obs = [];
    else
        obs = grid2world(map, [rows, cols]);
    end

    figure('Name',sprintf('Potential Field Map %d',map_id),'NumberTitle','off');
    show(map); hold on;
    title(sprintf('Potential Field — Map %d',map_id));
    plot(start_xy(1),start_xy(2),'go','MarkerSize',12,'MarkerFaceColor','g');
    plot(goal_xy(1), goal_xy(2),'r*','MarkerSize',14,'LineWidth',2);
    trl = plot(current(1),current(2),'m-','LineWidth',2);
    rbt = plot(current(1),current(2),'ko','MarkerSize',10,'MarkerFaceColor',[1 0.5 0]);

    for it = 1:max_it
        F_att = -k_att * (current - goal);
        F_rep = [0, 0];

        if ~isempty(obs)
            diffs = current - obs;
            dists = sqrt(sum(diffs.^2,2));
            in_range = dists < d0 & dists > 1e-3;
            if any(in_range)
                d_ = dists(in_range);
                dir_ = diffs(in_range,:);
                mag_ = k_rep .* (1./d_ - 1/d0) ./ (d_.^2) .* (1./d_);
                F_rep = sum(mag_ .* dir_, 1);
            end
        end

        F_tot = F_att + F_rep;
        fn = norm(F_tot);
        if fn < 1e-6
            break;
        end

        current = current + step * F_tot / fn;

        current(1) = max(0.1, min(24.9, current(1)));
        current(2) = max(0.1, min(24.9, current(2)));

        if checkOccupancy(map, current)
            current = current - 0.5 * step * F_tot / fn;
        end

        path(end+1,:) = current; %#ok<AGROW>

        if mod(it,5)==0
            set(trl,'XData',path(:,1),'YData',path(:,2));
            set(rbt,'XData',current(1),'YData',current(2));
            drawnow limitrate;
            pause(0.01);
        end

        if norm(current - goal) < tol
            success = true;
            break;
        end
    end

    drawnow;
end

%% ====================================================================
function p = CW2_find_free_point(map, p)
    p = double(p(:))';
    if ~checkOccupancy(map, p)
        return;
    end

    offsets = [
         0  0;
         1  0; -1  0; 0  1; 0 -1;
         1  1;  1 -1; -1  1; -1 -1;
         2  0; -2  0; 0  2; 0 -2;
         2  1;  2 -1; -2  1; -2 -1;
         1  2; -1  2;  1 -2; -1 -2
    ];

    for k = 1:size(offsets,1)
        q = p + offsets(k,:);
        if ~checkOccupancy(map, q)
            p = q;
            return;
        end
    end

    error('No free point found near [%.2f, %.2f].', p(1), p(2));
end

%% ====================================================================
function CW2_plot_comparison(results)
    n = 6;
    pl = zeros(1,n);
    tm = zeros(1,n);

    for i = 1:n
        r = results{i};
        pl(i) = r.path_length;
        tm(i) = r.time_elapsed;
    end

    pl_mat = reshape(pl, 3, 2);
    tm_mat = reshape(tm, 3, 2);

    figure('Name','A* vs Potential Field Comparison','NumberTitle','off','Position',[100 100 950 420]);

    subplot(1,2,1);
    b1 = bar(pl_mat,'grouped');
    b1(1).FaceColor = [0.2 0.5 0.8];
    b1(2).FaceColor = [0.9 0.5 0.1];
    set(gca,'XTickLabel',{'Map 1 (Simple)','Map 2 (Maze)','Map 3 (Warehouse)'});
    xlabel('Environment'); ylabel('Path Length (m)');
    title('Path Length'); grid on;
    legend({'A*','Potential Field'},'Location','northwest');

    subplot(1,2,2);
    b2 = bar(tm_mat,'grouped');
    b2(1).FaceColor = [0.2 0.5 0.8];
    b2(2).FaceColor = [0.9 0.5 0.1];
    set(gca,'XTickLabel',{'Map 1 (Simple)','Map 2 (Maze)','Map 3 (Warehouse)'});
    xlabel('Environment'); ylabel('Time (s)');
    title('Computation Time'); grid on;
    legend({'A*','Potential Field'},'Location','northeast');

    sgtitle('Section 6: A* vs Potential Field — Full Comparison','FontSize',13,'FontWeight','bold');
end
