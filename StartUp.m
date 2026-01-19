%clear; close; clc;
clc;
restoredefaultpath;

%% Initialize Values

project_start; % calls car_params.mlx
p = load('parameter.mat'); % calls car_params.mlx
assignin('base', 'p', p);

rrAppPath = strtrim(fileread('SelectedInstallationPath.txt'));
addpath(rrAppPath);

rrProjectPath = strtrim(fileread('SelectedProjectPath.txt'));

busData = load(fullfile(rrProjectPath, 'BusWorldToVehicleActors/3/BusWorldToVehicleActors.mat'));
varNames = fieldnames(busData);
for i = 1:numel(varNames)
    assignin('base', varNames{i}, busData.(varNames{i}));
end

rrScenarios = {};        % list of scenario names
idx = 1;

for i = 1:numel(p.SceneTypes)
    sceneName = p.SceneTypes{i};
    sceneName = sceneName(sceneName ~= ' '); % Clean whitespace
    
    % Notice: We loop through ObstacleTypes and use the same index for Locations
    for j = 1:numel(p.ObstacleTypes)
        for w = 1:numel(p.BehaviorTypes)
            % Build filename using index 'j' for both Type and Location
            rrScenarios{idx} = fullfile(rrProjectPath, sprintf('/Projects/%s/Scenarios/%s_%s_%s_%s.rrscenario', ...
                sceneName, sceneName, p.ObstacleTypes{j}, p.ObstacleLocations{j}, p.BehaviorTypes{w}));
            idx = idx + 1;
        end
    end
end

n = 1;

runs = repmat(struct('RunID', [], ...
                 'scenario',   "", ...
                 'TotalActors',  [], ...
                 'results',   [], ...
                 'failFlag', []), 1, n);

%% Open Model

prjFile = fullfile(rrProjectPath, 'Projects.prj'); 

if exist(prjFile, 'file')
    % This is the code equivalent of clicking "Open Project and Model"
    openProject(prjFile); 
else
    fprintf('Warning: .prj file not found in %s\n', rrProjectPath);
end

warning('off','all')
open(fullfile(rrProjectPath, "Misc Models", "ADC_RoadRunner.slx"))
set_param('ADC_RoadRunner', 'SolverType', 'Variable-step');
set_param('ADC_RoadRunner', 'Solver', 'ode23tb');
%% Open RoadRunner Project Files
for n = 1
%for n = 1:numel(rrScenarios)
    
    %Get Scenario Name Details
    [~, name, ext] = fileparts(rrScenarios{n}); % fileparts returns [path, name, extension]
    fileNameWithOutExt = [name]; % Just scenario file name
    fileNameWithOutExt = fileNameWithOutExt(fileNameWithOutExt ~= ' ');
    fileNameWithExt = [name, ext]; % Combine scenario name and extension
    fileNameWithExt = fileNameWithExt(fileNameWithExt ~= ' ');

    s = settings;
    s.roadrunner.application.InstallationFolder.TemporaryValue = rrAppPath;

    busInfoData = load('businfo.mat');
    varNamesBus = fieldnames(busInfoData);
    for i = 1:numel(varNamesBus)
        assignin('base', varNamesBus{i}, busInfoData.(varNamesBus{i}));
    end

    currentScenarioPath = rrScenarios{n};
    matchedScene = '';
    
    for s_idx = 1:numel(p.SceneTypes)
        % Check for both the version with spaces and without
        cleanName = p.SceneTypes{s_idx};
        cleanName = cleanName(cleanName ~= ' ');
        
        if contains(currentScenarioPath, cleanName) || contains(currentScenarioPath, p.SceneTypes{s_idx})
            % Use the original name from p.SceneTypes to match the folder on disk
            matchedScene = p.SceneTypes{s_idx}; 
            break;
        end
    end

    % Construct the path to the SPECIFIC scene project
    rrProjectPath_Final = fullfile(rrProjectPath, 'Projects', matchedScene);
    
    % Verify the path exists before calling roadrunner()
    if ~exist(rrProjectPath_Final, 'dir')
        error('Project folder not found: %s. Check if folder name matches GUI selection.', rrProjectPath_Final);
    end

    rrApp = roadrunner(rrProjectPath_Final);

    openScenario(rrApp,fileNameWithExt);
    rrSim = rrApp.createSimulation;
%% RoadRunner Scenario Data, Simulation Time

    set(rrSim, 'Logging','on');
    
    SimulationLength = 2;
    set(rrSim, MaxSimulationTime=SimulationLength);
    
    Ts = 0.05; 
    STEER_RATIO = -0.0582;
    LaneWidth = 3.85;

    assignin('base', 'Ts', Ts);
    assignin('base', 'STEER_RATIO', STEER_RATIO);
    assignin('base', 'LaneWidth', LaneWidth);
    
    
    % 1 2 3 4 ... 10 11 12: display values to check execution in helperSLAEBWithRRSetup script
    helperSLAEBWithRRSetup(rrApp, rrSim, scenarioFileName=fileNameWithOutExt)  % read scenario and create actorProfiles,cameraParams,radarParams
%% Simulation
    
    set(rrSim,SimulationCommand="Start")
    while strcmp(rrSim.get("SimulationStatus"), "Running")
      pause(1)
    end

    run('SavingSimulationData.m')
    run('CreatingPlots.m')

%% Close RoadRunner
    fprintf("Scenario " + n + " run")
    fprintf(fileNameWithOutExt)
    close(rrApp)

end