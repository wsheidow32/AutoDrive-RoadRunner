% Get logged results from scenario
simLog = get(rrSim,"SimulationLog");

% Discover all actors
actorIDs = get(simLog,"ActorIDs");
actorIDs = actorIDs(:)';
actorIDs = actorIDs(actorIDs > 0);
numActors = numel(actorIDs);

% Initializing
results = struct('ActorID',cell(1,numActors), ...
    'time',   [], ...
    'speed',  [], ...
    'posX',   [], ...
    'posY',   []);

for k = 1:numActors
    id = actorIDs(k);

    % Velocity log for this actor
    vLog = get(simLog,"Velocity","ActorID",id);
    if ~isempty(vLog)
        timeK  = [vLog.Time];
        speedK = arrayfun(@(x) norm(x.Velocity,2), vLog);
    else
        timeK  = [];
        speedK = [];
    end

    % Pose log for this actor (for XY path)
    pLog = get(simLog,"Pose","ActorID",id);
    if ~isempty(pLog)
        posXK = arrayfun(@(x) x.Pose(1,4), pLog);
        posYK = arrayfun(@(x) x.Pose(2,4), pLog);
    else
        posXK = [];
        posYK = [];
    end

    % Storing Data
    results(k).ActorID = id;
    results(k).time    = timeK;
    results(k).speed   = speedK;
    results(k).posX    = posXK';
    results(k).posY    = posYK';
end

% Storing data
runs(n).RunID = n;
runs(n).scenario = fileNameWithOutExt;
runs(n).TotalActors = actorIDs;
runs(n).results = results;

results_plots = runs(n).results;
numActors_plots = numel(results_plots);
actorIDs_plots  = [results_plots.ActorID];