%% ---- Config ----
actorA = 1;
actorB = 2;

THRESH_FT = 3;
THRESH_M  = THRESH_FT * 0.3048; % 0.9144 m

%% ---- Workspace inputs (safe defaults) ----
if exist('folderName','var') ~= 1 || isempty(folderName)
    folderName = 'Plots';
end
if exist('PassFolder','var') ~= 1 || isempty(PassFolder)
    PassFolder = 'Pass';
end
if exist('FailFolder','var') ~= 1 || isempty(FailFolder)
    FailFolder = 'Fail';
end
if exist('fileNameWithOutExt','var') ~= 1 || isempty(fileNameWithOutExt)
    fileNameWithOutExt = "Run";
end
if exist('n','var') ~= 1 || isempty(n)
    n = 1;
end
if exist('rrSim','var') ~= 1
    error("CollisionDetection_Actor1_Actor2.m requires rrSim in workspace.");
end


passDir = fullfile(folderName, PassFolder);
failDir = fullfile(folderName, FailFolder);

%% ---- Pull logs ----
simLog = rrSim.get("SimulationLog");

pA = get(simLog, "Pose", "ActorID", actorA);
pB = get(simLog, "Pose", "ActorID", actorB);

if isempty(pA) || isempty(pB)
    % If either actor is missing, do nothing (or fail). Here: fail-safe into Fail folder.
    PassFail = 'Fail';
    outDir = failDir;

    fig = figure('Visible','off');
    axis off;
    title(sprintf('Collision Distance A%d-B%d | Run %d | %s', actorA, actorB, n, PassFail));
    text(0.1, 0.5, 'Missing Pose logs for one or both actors.', 'FontSize', 12);
    saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_CollisionDistance_', PassFail, '.png']));
    close(fig);
    return;
end

tA = [pA.Time]';
xA = arrayfun(@(e) e.Pose(1,4), pA)';
yA = arrayfun(@(e) e.Pose(2,4), pA)';

tB = [pB.Time]';
xB = arrayfun(@(e) e.Pose(1,4), pB)';
yB = arrayfun(@(e) e.Pose(2,4), pB)';

%% ---- Align time bases ----
t0 = max(tA(1), tB(1));
t1 = min(tA(end), tB(end));

if t1 <= t0
    PassFail = 'Fail';
    outDir = failDir;

    fig = figure('Visible','off');
    axis off;
    title(sprintf('Collision Distance A%d-B%d | Run %d | %s', actorA, actorB, n, PassFail));
    text(0.1, 0.5, 'No overlapping time interval between actor logs.', 'FontSize', 12);
    saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_CollisionDistance_', PassFail, '.png']));
    close(fig);
    return;
end

% Use Actor A time samples within overlap as the reference
t = tA(tA >= t0 & tA <= t1);
if numel(t) < 3
    % Fallback to a merged, sorted time base
    t = unique([tA; tB]);
    t = t(t >= t0 & t <= t1);
end

xAi = interp1(tA, xA, t, 'linear', 'extrap');
yAi = interp1(tA, yA, t, 'linear', 'extrap');
xBi = interp1(tB, xB, t, 'linear', 'extrap');
yBi = interp1(tB, yB, t, 'linear', 'extrap');

%% ---- Distance + strict threshold ----
dist_m  = hypot(xAi - xBi, yAi - yBi);
dist_ft = dist_m / 0.3048;

violCount = nnz(dist_m < THRESH_M);
if violCount > 0
    PassFail = 'Fail';
    outDir = failDir;
else
    PassFail = 'Pass';
    outDir = passDir;
end

%% ---- Plot & save ----
fig = figure('Visible','off');
hold on; grid on;

plot(t, dist_ft, 'LineWidth', 1.5);
yline(THRESH_FT, 'r', 'Threshold (3 ft)', 'LabelHorizontalAlignment', 'left');

xlabel('Time (s)');
ylabel('Distance (ft)');
title(sprintf('Actor %d to Actor %d Distance | Run %d', actorA, actorB, n));

saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_CollisionDistance_', PassFail, '.png']));
close(fig);