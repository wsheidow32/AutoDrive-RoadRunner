%% ---- inputs ---- 
targetActorID = 1;
accelerationFlag = 0;
curvatureFlag = 0;
jerkFlag = 0;

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

passDir = fullfile(folderName, PassFolder);
failDir = fullfile(folderName, FailFolder);

%% ---- Thresholds (strict: any exceedance => FAIL) ----
A_LON = 1.47;  % m/s^2
A_LAT = 2.16;  % m/s^2
JERK  = 1.00;  % m/s^3
KAPPA = 0.15;  % 1/m
DKDS  = 1.18;  % 1/m^2

%% ---- Compute signals ----
simLog = rrSim.get("SimulationLog");
sig = compute_smoothness_signals_local(simLog, targetActorID);
t = sig.t;

%% ---- Independent evaluations ----
% 1) a_lon
viol_a_lon = nnz(abs(sig.alon) > A_LON);
if viol_a_lon == 0
    pf_a_lon = 'Pass';
    dir_a_lon = passDir;
else
    pf_a_lon = 'Fail';
    dir_a_lon = failDir;
    accelerationFlag = 1;
    failFlag = 1;
end

% 2) a_lat
viol_a_lat = nnz(abs(sig.alat) > A_LAT);
if viol_a_lat == 0
    pf_a_lat = 'Pass';
    dir_a_lat = passDir;
else
    pf_a_lat = 'Fail';
    dir_a_lat = failDir;
    accelerationFlag = 1;
    failFlag = 1;
end

% 3) jerk
viol_jerk = nnz(abs(sig.jerk) > JERK);
if viol_jerk == 0
    pf_jerk = 'Pass';
    dir_jerk = passDir;
else
    pf_jerk = 'Fail';
    dir_jerk = failDir;
    jerkFlag = 1;
    failFlag = 1;
end

% 4) curvature kappa
viol_kappa = nnz(abs(sig.kappa) > KAPPA);
if viol_kappa == 0
    pf_kappa = 'Pass';
    dir_kappa = passDir;
else
    pf_kappa = 'Fail';
    dir_kappa = failDir;
    curvatureFlag = 1;
    failFlag = 1;
end

% 5) curvature rate dkds
viol_dkds = nnz(abs(sig.dkds) > DKDS);
if viol_dkds == 0
    pf_dkds = 'Pass';
    dir_dkds = passDir;
else
    pf_dkds = 'Fail';
    dir_dkds = failDir;
    curvatureFlag = 1;
    failFlag = 1;
end

%% ---- Plot 1: Longitudinal Acceleration ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.alon, 'LineWidth', 1.5);
yline(+A_LON, '--', 'thr +'); yline(-A_LON, '--', 'thr -');
title("Trajectory Smoothness - Long Accel | Run " + n)
xlabel("Time (s)")
ylabel("a_{lon} (m/s^2)")
saveas(fig, fullfile(dir_a_lon, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_LongAccel_', pf_a_lon, '.png']));
close(fig);

%% ---- Plot 2: Lateral Acceleration ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.alat, 'LineWidth', 1.5);
yline(+A_LAT, '--', 'thr +'); yline(-A_LAT, '--', 'thr -');
title("Trajectory Smoothness - Lat Accel | Run " + n)
xlabel("Time (s)")
ylabel("a_{lat} (m/s^2)")
saveas(fig, fullfile(dir_a_lat, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_LatAccel_', pf_a_lat, '.png']));
close(fig);

%% ---- Plot 3: Total Jerk ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.jerk, 'LineWidth', 1.5);
yline(+JERK, '--', 'thr +'); yline(-JERK, '--', 'thr -');
title("Trajectory Smoothness - Jerk | Run " + n)
xlabel("Time (s)")
ylabel("jerk (m/s^3)")
saveas(fig, fullfile(dir_jerk, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_Jerk_', pf_jerk, '.png']));
close(fig);

%% ---- Plot 4: Curvature κ ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.kappa, 'LineWidth', 1.5);
yline(+KAPPA, '--', 'thr +'); yline(-KAPPA, '--', 'thr -');
title("Trajectory Smoothness - Curvature | Run " + n)
xlabel("Time (s)")
ylabel("\kappa (1/m)")
saveas(fig, fullfile(dir_kappa, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_Curvature_', pf_kappa, '.png']));
close(fig);

%% ---- Plot 5: Curvature Rate dκ/ds ----
fig = figure('Visible','off');
hold on; grid on;
plot(t, sig.dkds, 'LineWidth', 1.5);
yline(+DKDS, '--', 'thr +'); yline(-DKDS, '--', 'thr -');
title("Trajectory Smoothness - Curvature Rate | Run " + n)
xlabel("Time (s)")
ylabel("d\kappa/ds (1/m^2)")
saveas(fig, fullfile(dir_dkds, [char(fileNameWithOutExt), '_', num2str(n), '_Smoothness_CurvatureRate_', pf_dkds, '.png']));
close(fig);

% If FailFlag has been triggered data will be saved
if failFlag == 1
    TotalFails = TotalFails + 1;
    fails(TotalFails).RunID = n;
    fails(TotalFails).scenario = fileNameWithOutExt;
    fails(TotalFails).TotalActors = actorIDs;
    fails(TotalFails).results = results;
    fails(TotalFails).Acceleration = accelerationFlag;
    fails(TotalFails).Curvature = curvatureFlag;
    fails(TotalFails).Jerk = jerkFlag;
    fails(TotalFails).LanePosition = laneFlag;
end
runs(n).failFlag = failFlag;
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
