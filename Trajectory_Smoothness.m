
%% ---- Config ----
targetActorID = 1;

% Thresholds
A_LON = 1.47;  % m/s^2
A_LAT = 2.16;  % m/s^2
JERK  = 1.00;  % m/s^3
DKDS  = 1.18;  % 1/m^2

% Display control (set by CreatingPlots.m)
if exist('showFigures','var') ~= 1 || isempty(showFigures)
    showFigures = false;
end
vis = ternary_local(showFigures, 'on', 'off');

% Output folder
if exist('folderName','var') ~= 1 || isempty(folderName)
    folderName = 'Plots';
end
if ~exist(folderName, 'dir')
    mkdir(folderName);
end

% Filename prefix
if exist('fileNameWithOutExt','var') ~= 1 || isempty(fileNameWithOutExt)
    fileNameWithOutExt = "Run";
end
if exist('n','var') ~= 1 || isempty(n)
    n = 1;
end
prefix = string(fileNameWithOutExt);
runStr = "_" + string(n);

%% ---- Pull sim log ----
simLog = rrSim.get("SimulationLog");

%% ---- Compute signals ----
sig = compute_smoothness_signals_local(simLog, targetActorID);
t = sig.t;

%% ---- Plot 1: Long & Lat acceleration (+ thresholds) ----
fig = figure('Visible', vis, 'Color', 'w');
hold on; grid on;

plot(t, sig.alon, 'LineWidth', 1.5);
plot(t, sig.alat, 'LineWidth', 1.5);

% Threshold bands
yline(+A_LON, '--', 'a_{lon} +thr');
yline(-A_LON, '--', 'a_{lon} -thr');
yline(+A_LAT, ':',  'a_{lat} +thr');
yline(-A_LAT, ':',  'a_{lat} -thr');

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title(sprintf('Trajectory Smoothness (Actor %d) - Accel | Run %d', targetActorID, n));
legend('a_{lon}', 'a_{lat}', 'Location', 'best');

saveas(fig, fullfile(folderName, sprintf('%s%s_Smoothness_Accel.png', prefix, runStr)));
if ~showFigures, close(fig); end

%% ---- Plot 2: Total jerk (+ threshold) ----
fig = figure('Visible', vis, 'Color', 'w');
grid on; hold on;

plot(t, sig.jerk, 'LineWidth', 1.5);
yline(+JERK, '--', 'jerk +thr');
yline(-JERK, '--', 'jerk -thr');

xlabel('Time (s)');
ylabel('Jerk (m/s^3)');
title(sprintf('Trajectory Smoothness (Actor %d) - Jerk | Run %d', targetActorID, n));

saveas(fig, fullfile(folderName, sprintf('%s%s_Smoothness_Jerk.png', prefix, runStr)));
if ~showFigures, close(fig); end

%% ---- Plot 3: Curvature + Curvature rate (+ DKDS threshold on right axis) ----
fig = figure('Visible', vis, 'Color', 'w');
grid on; hold on;

yyaxis left
plot(t, sig.kappa, 'LineWidth', 1.5);
ylabel('\kappa (1/m)');

yyaxis right
plot(t, sig.dkds, 'LineWidth', 1.5);
yline(+DKDS, '--', 'd\kappa/ds +thr');
yline(-DKDS, '--', 'd\kappa/ds -thr');
ylabel('d\kappa/ds (1/m^2)');

xlabel('Time (s)');
title(sprintf('Trajectory Smoothness (Actor %d) - Curvature | Run %d', targetActorID, n));

saveas(fig, fullfile(folderName, sprintf('%s%s_Smoothness_Curvature.png', prefix, runStr)));
if ~showFigures, close(fig); end

%% =========================
%% Local helper functions
%% =========================
function sig = compute_smoothness_signals_local(simLog, actorID)
vLog = get(simLog, "Velocity", "ActorID", actorID);
pLog = get(simLog, "Pose",     "ActorID", actorID);

if isempty(vLog) && isempty(pLog)
    error("No Pose/Velocity logs found for ActorID=%d", actorID);
end

% Velocity
if ~isempty(vLog)
    velTime = [vLog.Time]';
    vx0 = arrayfun(@(e) e.Velocity(1), vLog)';
    vy0 = arrayfun(@(e) e.Velocity(2), vLog)';
else
    velTime = []; vx0 = []; vy0 = [];
end

% Pose
if ~isempty(pLog)
    poseTime = [pLog.Time]';
    x0 = arrayfun(@(e) e.Pose(1,4), pLog)';
    y0 = arrayfun(@(e) e.Pose(2,4), pLog)';
    yaw0 = arrayfun(@(e) atan2(e.Pose(2,1), e.Pose(1,1)), pLog)';
else
    poseTime = []; x0 = []; y0 = []; yaw0 = [];
end

% Choose time base (prefer pose)
if ~isempty(poseTime)
    t = poseTime;
    x = x0; y = y0;

    vx_i = interp1_safe_local(velTime, vx0, t);
    vy_i = interp1_safe_local(velTime, vy0, t);

    vx = fallback_local(vx_i, gradient(x, t));
    vy = fallback_local(vy_i, gradient(y, t));

    yaw = yaw0;
    if isempty(yaw) || numel(yaw) ~= numel(t)
        yaw = atan2(vy, vx);
    end
else
    t = velTime;
    vx = vx0; vy = vy0;
    x = nan(size(t)); y = nan(size(t));
    yaw = atan2(vy, vx);
end

yaw = unwrap(yaw);

% Smoothing
smoothWinSec = 0.3;
dt = median(diff(t));
win = max(5, 2*floor((smoothWinSec/dt - 1)/2)+1);

vx = movmean(vx, win);
vy = movmean(vy, win);
v  = hypot(vx, vy);

ax = movmean(gradient(vx, t), win);
ay = movmean(gradient(vy, t), win);

[alon, alat] = world2body_local(ax, ay, yaw);

jx = movmean(gradient(ax, t), win);
jy = movmean(gradient(ay, t), win);
jerk = hypot(jx, jy);

% Curvature + dκ/ds
if ~isempty(x0) && numel(x0) == numel(t) && all(isfinite(x))
    kappa = curvature_local(vx, vy, ax, ay);
else
    kappa = movmean((gradient(yaw, t)) ./ max(v, 0.05), win);
end
dk_dt = gradient(kappa, t);
dkds  = dk_dt ./ max(v, 0.05);

sig = struct('t',t,'x',x,'y',y,'v',v,'alon',alon,'alat',alat,'jerk',jerk,'kappa',kappa,'dkds',dkds);
end

function out = fallback_local(primary, secondary)
if isempty(primary), out = secondary; else, out = primary; end
end

function v = interp1_safe_local(t_src, x_src, t_tar)
if isempty(t_src) || isempty(x_src), v = []; return; end
v = interp1(t_src, x_src, t_tar, 'linear', 'extrap');
end

function [alon, alat] = world2body_local(ax, ay, yaw)
c = cos(yaw); s = sin(yaw);
alon =  ax.*c + ay.*s;
alat = -ax.*s + ay.*c;
end

function k = curvature_local(dx, dy, ddx, ddy)
num = dx.*ddy - dy.*ddx;
den = (dx.^2 + dy.^2).^(3/2);
k = num ./ max(den, 1e-6);
end

function out = ternary_local(cond, a, b)
if cond, out = a; else, out = b; end
end
