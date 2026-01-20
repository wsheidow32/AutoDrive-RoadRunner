% Naming Folders
folderName = 'Plots';
PassFolder = 'Pass';
FailFolder = 'Fail';
OtherFolder = "Speed and Map";

% Creating Folders
mkdir(folderName, PassFolder)
mkdir(folderName, FailFolder)
mkdir(folderName, OtherFolder)

% Speed vs Time Graph
fig = figure('Visible', 'off');
hold on;
grid on;
for k = 1:numActors_plots
    t = results_plots(k).time;
    v = results_plots(k).speed;
    if ~isempty(t) && ~isempty(v)
        plot(t, v, 'LineWidth', 1.5);
    end
end
title("Actor Speeds from RoadRunner Scenario | Run " + n)
xlabel("Time (s)")
ylabel("Velocity (m/s)")
if ~isempty(actorIDs_plots)
    legend(compose("Actor ID = %d", actorIDs_plots), 'Location','best')
end

% Save the plot as a PNG file
saveas(fig, fullfile(fullfile(folderName, OtherFolder), [fileNameWithOutExt, '_', num2str(n), '_Speeds', '.png']));

% HD Map Lanes
hdMap = get(rrSim, "Map");
lanes = hdMap.Lanes;

% Turning Display Off
fig = figure('Visible', 'off');
hold on;
grid on;

% draw lanes
laneHandle = [];
for i = 1:numel(lanes)
    cp = lanes(i).Geometry;
    h = plot(cp(:,1), cp(:,2), 'k', 'LineWidth', 1);
    if i == 1
        laneHandle = h;
    end
end

% overlay each actor trajectory
hActors = gobjects(numActors_plots,1);
for k = 1:numActors_plots
    x = results_plots(k).posX;
    y = results_plots(k).posY;
    if ~isempty(x) && ~isempty(y)
        hActors(k) = plot(x, y, 'LineWidth', 2);
    end
end

axis equal
title("Actor Positions from RoadRunner Scenario | Run " + n)
xlabel("X (m)"); ylabel("Y (m)")

% Create legend with lanes and actors
if ~isempty(laneHandle) && numActors_plots > 0
    legend([laneHandle; hActors], ["Lanes", compose("Actor ID = %d", actorIDs_plots)], ...
        'Location','bestoutside');
end

% Save the plot as a PNG file
saveas(fig, fullfile(fullfile(folderName, OtherFolder), [fileNameWithOutExt, '_', num2str(n), '_Map', '.png']));

PositionError = out.PositionError;

% Creating Lane Position Error Plot
fig = figure('Visible', 'off');
plot(PositionError.Time, PositionError.Data, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Error vs Time');
yline(0.3, 'r', 'Threshold Value');

% Variable to Save Fail
failFlag = 0;

% Checking Threshold
if any(abs(PositionError.Data) > 0.3)
    PassFail = 'Fail';
    failFlag = 1;

    % Adding to Fail Matrix
    TotalFails = TotalFails + 1;
    fails(TotalFails).LanePosition = 1;

else
    PassFail = 'Pass';
end

runs(n).failFlag = failFlag;

saveas(fig, fullfile(fullfile(folderName, PassFail), [fileNameWithOutExt, '_', num2str(n), '_LanePosition_', PassFail, '.png']));