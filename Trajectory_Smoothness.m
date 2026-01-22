targetActorID = 1;

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
    error("TrajectorySmoothnessPlots_Actor1.m requires rrSim in workspace.");
end

mkdir(folderName, PassFolder);
mkdir(folderName, FailFolder);

passDir = fullfile(folderName, PassFolder);
failDir = fullfile(folderName, FailFolder);

%% ---- Thresholds (strict fail: ANY exceedance fails) ----
A_LON = 1.47;  % m/s^2
A_LAT = 2.16;  % m/s^2
JERK  = 1.00;  % m/s^3
DKDS  = 1.18;  % 1/m^2

%% ---- Compute signals ----
simLog = rrSim.get("SimulationLog");
sig = compute_smoothness_signals_local(simLog, targetActorID);

viol_a_lon = nnz(abs(sig.alon)  > A_LON);
viol_a_lat = nnz(abs(sig.alat)  > A_LAT);
viol_jerk  = nnz(abs(sig.jerk)  > JERK);
viol_dkds  = nnz(abs(sig.dkds)  > DKDS);

smoothFail = (viol_a_lon > 0) || (viol_a_lat > 0) || (viol_jerk > 0) || (viol_dkds > 0);

PassFailSmooth = 'Pass';
outDir = passDir;
if smoothFail
    PassFailSmooth = 'Fail';
    outDir = failDir;
end

fprintf("\nTrajectory Smoothness (Actor %d): %s | viol(a_lon)=%d viol(a_lat)=%d viol(jerk)=%d viol(dkds)=%d\n", ...
    targetActorID, PassFailSmooth, viol_a_lon, viol_a_lat, viol_jerk, viol_dkds);

t = sig.t;

%% ---- Plot 1: Longitudinal Acceleration ( acceleration along vehicle's forward direction) ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.alon, 'LineWidth', 1.5);
yline(+A_LON, '--', 'thr +'); yline(-A_LON, '--', 'thr -');
title("Trajectory Smoothness - Long Accel | Run " + n)
xlabel("Time (s)")
ylabel("a_{lon} (m/s^2)")
saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_LongAccel_', PassFailSmooth, '.png']));
close(fig);

%% ---- Plot 2: Lateral Acceleration (side-to-side accceleration / ~ corning force) ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.alat, 'LineWidth', 1.5);
yline(+A_LAT, '--', 'thr +'); yline(-A_LAT, '--', 'thr -');
title("Trajectory Smoothness - Lat Accel | Run " + n)
xlabel("Time (s)")
ylabel("a_{lat} (m/s^2)")
saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_LatAccel_', PassFailSmooth, '.png']));
close(fig);

%% ---- Plot 3: Total Jerk (how quickly acceleration changes) ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.jerk, 'LineWidth', 1.5);
yline(+JERK, '--', 'thr +'); yline(-JERK, '--', 'thr -');
title("Trajectory Smoothness - Jerk | Run " + n)
xlabel("Time (s)")
ylabel("jerk (m/s^3)")
saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_Jerk_', PassFailSmooth, '.png']));
close(fig);

%% ---- Plot 4: Curvature κ (road-wheel steering angle / how much you're turning per meter) ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.kappa, 'LineWidth', 1.5);
title("Trajectory Smoothness - Curvature | Run " + n)
xlabel("Time (s)")
ylabel("\kappa (1/m)")
saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_Curvature_', PassFailSmooth, '.png']));
close(fig);

%% ---- Plot 5: Curvature Rate dκ/ds (steering rate per meter traveled / how quickly steering is changing as u move forward) ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.dkds, 'LineWidth', 1.5);
yline(+DKDS, '--', 'thr +'); yline(-DKDS, '--', 'thr -');
title("Trajectory Smoothness - Curvature Rate | Run " + n)
xlabel("Time (s)")
ylabel("d\kappa/ds (1/m^2)")
saveas(fig, fullfile(outDir, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_CurvatureRate_', PassFailSmooth, '.png']));
close(fig);

%% ---- Local helper functions ----
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

% Prefer pose time base
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
