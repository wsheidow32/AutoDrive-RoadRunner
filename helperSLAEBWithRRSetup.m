function helperSLAEBWithRRSetup(rrAppObj, scenarioSimulationObj, nvp)
%helperSLAEBWithRRSetup create required buses, variables for simulating the
% Autonomous Emergency Braking (AEB) with RoadRunner Scenario Example.
%
% Initializes the AEBWithRRTestBench.slx model by creating data in base
% workspace.
%
% Optional inputs
%   scenarioFcnName:
%     - Name of function which returns scenario which is
%       compatible with AEBWithRRTestBench.slx
%     - Valid values are:
%           "scenario_01_CarToCar_FrontTAP.rrscenario"
%           "scenario_02_CarToCar_RearStationary.rrscenario"
%
% Examples of calling this function:
%
%    helperSLAEBWithRRSetup(rrAppObj, scenarioSimulationObj, scenarioFcnName= "scenario_01_CarToCar_FrontTAP")
%    helperSLAEBWithRRSetup(rrAppObj, scenarioSimulationObj, scenarioFcnName= "scenario_02_CarToCar_RearStationary")
%    helperSLAEBWithRRSetup(rrAppObj, scenarioSimulationObj, scenarioFcnName= "scenario_03_CarToPedestrian_NearSide_Adult")
%
% This helper function initializes the AEB example model. It loads
% necessary control constants and sets up the buses required for the
% referenced models
%
%   This is a helper function for example purposes and may be removed or
%   modified in the future.

% Copyright 2022 The MathWorks, Inc.

arguments
    rrAppObj = [];
    scenarioSimulationObj =[];
    nvp.scenarioFileName = "TrafficLight_LeftTurn"
end

scenarioFileName = nvp.scenarioFileName;
assignin('base', 'scenarioFileName', scenarioFileName);

% Initialize simulation sample time
Ts = 0.05;
assignin('base', 'Ts', Ts); % Simulation sample time  (s)

egoActorID = 1;
egoSetSpeed = 2.778; % default speed

egoInitialPose = struct;
egoInitialPose.ActorID = 1;
egoInitialPose.Position = [0 0 0];
egoInitialPose.Velocity = [0 0 0];
egoInitialPose.Roll = 0;
egoInitialPose.Pitch = 0;
egoInitialPose.Yaw = 0;
egoInitialPose.AngularVelocity = [0 0 0];

numActors = 1;

initActorProfile = struct(...
    'ActorID',1,...
    'ClassID',1,...
    'Length',4.7,...
    'Width',1.8,...
    'Height',1.4,...
    'OriginOffset',[0 0 0],...
    'FrontOverhang',0,...
    'RearOverhang',0,...
    'Wheelbase',0,...
    'Color',[0 0 0]);

actorProfiles = repmat(initActorProfile, 1, numActors);

if ~isempty(scenarioSimulationObj)
    p = [1 2 3 4];
    disp(p)
    % Get current RoadRunner application status
    rrStatus = rrAppObj.status();
    % Get the current scenario status
    scenarioStatus = rrStatus.Scenario;

    % If the scenario status from the RoadRunner has empty then open the
    % scenario specified from the input argument.
    if ~isempty(scenarioStatus)
         p = [5 6 7 8];             %% checking exec
        disp(p)
        % Get the file name from scenarioStatus.
        [~,fileName,~] = fileparts(scenarioStatus.Filename);
         % Open scenario if it is not already opened
        if(fileName ~= scenarioFileName)
            disp(p)
            openScenario(rrAppObj, strjoin([scenarioFileName, '.rrscenario'],""));
        end
    else
        % Assign the default file Name variable with the input scenario
        % file name.
        fileName = scenarioFileName;
        openScenario(rrAppObj, strjoin([fileName, '.rrscenario'],""));
    end

    % Read actor profiles from RoadRunner Scenario
    worldActor = scenarioSimulationObj.getScenario();
    world = worldActor.actor_spec.world_spec;
     p = [9 10 11 12];   %% checking exec
     disp(p)     
    numActors = length(world.actors);
    actorProfiles = helperGetActorProfiles(world.actors,OriginOffset="RearAxleCenter");

    % Get ego ActorID
    for i = 1:length(world.behaviors)
        if contains(upper(world.behaviors(i).asset_reference),upper('NewBehav.rrbehavior')) %behavior name
            egoBehavior = world.behaviors(1).id;
            break;
        end
    end
    for i = 1:length(world.actors)
        id = str2double(world.actors(i).actor_spec.id);
        if isequal(world.actors(i).actor_spec.behavior_id,egoBehavior)
            egoActorID = id;
        end
    end

    egoSetSpeed = str2double(rrAppObj.getScenarioVariable('Ego_Speed')); % get speed, m/s  % VUT_Speed -> Ego_Speed
    egoInitialPose = helperGetActorPose(world.actors, egoActorID, egoSetSpeed);
end

% Create AEB buses
%helperCreateAEBBusObjects("numTargetActors",numActors-1,"refPathSize",1000);
evalin('base', sprintf('helperCreateAEBBusObjects("numTargetActors", %d, "refPathSize", 1000)', numActors-1));

assignin('base', 'maxNumActors', numActors);
assignin('base', 'numTargetActors',  numActors-1);
assignin('base', 'egoActorID', egoActorID);
assignin('base', 'egoInitialPose', egoInitialPose);

% Arc length between interpolated ego path points
DiscretizationDistance = 0.2;
assignin('base','DiscretizationDistance', DiscretizationDistance);

%%
% To reduce command-window output, turn off the MPC update messages.
%%
mpcverbosity("off");

%% General Model parameters

% AEBControllerStepSize = Ts;
% assignin('base','AEBControllerStepSize', AEBControllerStepSize); % Step size for AEB Controller (s)
% 
% NLMPCTs = Ts;
% assignin('base','NLMPCTs', NLMPCTs); % Sample time for NLMPC Controller (s)

%% Sensor configuration
% Long Range Radar
radarParams.azRes = 4;
radarParams.rangeRes = 2.5;
radarParams.rangeRateRes = 0.5;
radarParams.radarRange = 174;
radarParams.azFov = 20;
assignin('base','radarParams', radarParams);

% 1.2MP, FoV = 49 deg
cameraParams.focalLength = [800, 800];
cameraParams.principalPoint = [320, 240];
cameraParams.imageSize = [480, 640];
cameraParams.cameraRange = 150;
cameraParams.pitchAngle = 10;
assignin('base','cameraParams', cameraParams);

% Tracking and Sensor Fusion Parameters                        Units
trackingParams.clusterSize = 4;        % Distance for clustering               (m)
trackingParams.assigThresh = 50;      % Tracker assignment threshold          (N/A)
trackingParams.M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
trackingParams.N           = 3;        % Tracker N value for M-out-of-N logic  (N/A)
trackingParams.numCoasts   = 3;        % Number of track coasting steps        (N/A)
trackingParams.numTracks   = 20;       % Maximum number of tracks              (N/A)
trackingParams.numSensors  = 2;        % Maximum number of sensors             (N/A)

% % 
% % Position and velocity selectors from track state
% % The filter initialization function used in this example is initcvekf that
% % defines a state that is: [x;vx;y;vy;z;vz].
trackingParams.posSelector = [1,0,0,0,0,0; 0,0,1,0,0,0]; % Position selector   (N/A)
% 
% % Assign TrackingParams struct in base workspace
assignin('base','trackingParams',trackingParams);

%% Controller parameters
% maxSteer = 1.13; % Maximum steering angle (rad)
% assignin('base', 'maxSteer', maxSteer);
% 
% minSteer = -1.13; % Minimum steering angle (rad)
% assignin('base', 'minSteer', minSteer);
% 
% min_ac = -3;
% assignin('base','min_ac', min_ac);      % Minimum acceleration   (m/s^2)
% 
% max_ac = 3;
% assignin('base', 'max_ac', max_ac);     % Maximum acceleration   (m/s^2)
% 
% predictionHorizon = 10; % Number of steps for preview    (N/A)
% assignin('base', 'predictionHorizon', predictionHorizon);
% 
% controlHorizon = 2;  % The number of MV moves to be optimized at control interval. (N/A)
% assignin('base', 'controlHorizon', controlHorizon);
% 
% assignin('base','max_dc', -9.8); % Maximum deceleration   (m/s^2)

%% FCW parameters
% FCW.timeToReact  = 1.2;         % driver reaction time                   (sec)
% FCW.driver_decel = 4.0;         % driver braking deceleration            (m/s^2)
% 
% % Assign FCW struct in base workspace
% assignin('base','FCW',FCW);
% 
% % AEB parameters
% AEB.PB1_decel = 3.8;            % 1st stage Partial Braking deceleration (m/s^2)
% AEB.PB2_decel = 5.3;            % 2nd stage Partial Braking deceleration (m/s^2)
% AEB.FB_decel  = 9.8;            % Full Braking deceleration              (m/s^2)
% AEB.headwayOffset = 3.7;        % headway offset                         (m)
% AEB.timeMargin = 0.08;             % headway time margin                    (sec)
% 
% % Assign AEB struct in base workspace
% assignin('base','AEB',AEB);
% 
% egoVehDyn = egoVehicleDynamicsParams(egoInitialPose, actorProfiles(egoActorID));
% % Vehicle Parameters
% % Dynamics modeling parameters
% m       = 1575;                         % Total mass of vehicle                          (kg)
% Iz      = 2875;                         % Yaw moment of inertia of vehicle               (m*N*s^2)
% lf      = egoVehDyn.CGToFrontAxle;      % Longitudinal distance from c.g. to front axle (m)
% lr      = egoVehDyn.CGToRearAxle;       % Longitudinal distance from c.g. to rear axle  (m)
% Cf      = 19000;                        % Cornering stiffness of front tires             (N/rad)
% Cr      = 33000;                        % Cornering stiffness of rear tires              (N/rad)
% tau     = 0.5;                          % Longitudinal time constant (throttle)          (N/A)
% tau2    = 0.07;                         % Longitudinal time constant (brake)             (N/A)


%% NLMPC params

% create nlmpc object with 7 prediction model states, 3 prediction model
% outputs.
% nlobj = nlmpc(7,3,'MV',[1 2],'MD',3,'UD',4);
% % two MV (Manipulated Variables) signals: acceleration and steering.
% % measured disturbance (MD) : the product of the road curvature and the longitudinal velocity
% % unmeasured disturbance (UD) : white noise.
% 
% % Specify the controller sample time, prediction horizon, and control
% % horizon.
% nlobj.Ts = Ts;
% nlobj.PredictionHorizon = predictionHorizon;
% nlobj.ControlHorizon = controlHorizon;
% 
% % Specify the state function for the nonlinear plant model and its
% % Jacobian.
% nlobj.Model.StateFcn = "helperNLMPCStateFcn";
% nlobj.Jacobian.StateFcn = "helperNLMPCStateJacFcn";
% 
% %
% % Specify the output function for the nonlinear plant model and its
% % Jacobian. The output variables are:
% %
% % * Longitudinal velocity
% % * Lateral deviation
% % * Sum of the yaw angle and yaw angle output disturbance
% %
% nlobj.Model.OutputFcn = "helperNLMPCOutputFcn";
% nlobj.Jacobian.OutputFcn = "helperNLMPCJacOutputFcn";
% 
% %
% % Set the constraints for manipulated variables.
% nlobj.MV(1).Min = min_ac;       % Maximum acceleration (m/s^2)
% nlobj.MV(1).Max = max_ac;       % Minimum acceleration  (m/s^2)
% nlobj.MV(2).Min = minSteer;     % Minimum steering angle  (rad)
% nlobj.MV(2).Max = maxSteer;     % Maximum steering angle  (rad)
% 
% % Set the scale factors.
% nlobj.OV(1).ScaleFactor = 15;   % Typical value of longitudinal velocity
% nlobj.OV(2).ScaleFactor = 0.5;  % Range for lateral deviation
% nlobj.OV(3).ScaleFactor = 0.5;  % Range for relative yaw angle
% nlobj.MV(1).ScaleFactor = 6;    % Range of steering angle
% nlobj.MV(2).ScaleFactor = 2.26; % Range of acceleration
% nlobj.MD(1).ScaleFactor = 0.2;  % Range of Curvature
% 
% % Specify the weights in the standard MPC cost function. The third output,
% % yaw angle, is allowed to float because there are only two manipulated
% % variables to make it a square system. In this example, there is no
% % steady-state error in the yaw angle as long as the second output, lateral
% % deviation, reaches 0 at steady state.
% nlobj.Weights.OutputVariables = [1 1 0];
% 
% % Penalize acceleration change more for smooth driving experience.
% nlobj.Weights.ManipulatedVariablesRate = [0.3 0.1];
% 
% params = {m,Iz,Cf,Cr,lf,lr,tau}';
% nlobj.Model.NumberOfParameters = numel(params);
% 
% % Load controller test bench model
% controllerRefModel = 'AEBController';
% wasRefModelLoaded = bdIsLoaded(controllerRefModel);
% if ~wasRefModelLoaded
%     load_system(controllerRefModel)
% end
% mdl = [controllerRefModel,  '/NLMPC Controller/Nonlinear MPC Controller'];
% 
% % clear and create parasBusObject if it exists
% evalin( 'base', 'clear(''paramsBusObject'')' );
% createParameterBus(nlobj, [mdl '/Nonlinear MPC Controller'], 'paramsBusObject', params);
% bdclose(mdl);
% 
% assignin('base', 'nlobj', nlobj);
% assignin('base','egoVehDyn',egoVehDyn);

%% Assign vehicle dynamics modeling parameters to base work space
% assignin('base', 'm',    m);
% assignin('base', 'Iz',   Iz);
% assignin('base', 'Cf',   Cf);
% assignin('base', 'Cr',   Cr);
% assignin('base', 'lf',   lf);
% assignin('base', 'lr',   lr);
% assignin('base', 'tau',  tau);
% assignin('base', 'tau2', tau2);
% assignin('base','egoVehDyn',egoVehDyn);

% Goal for collision mitigation >= 90%
safetyGoal = 90;
assignin('base','safetyGoal', safetyGoal);

% set ego velocity (m/s)
assignin('base','v_set', egoSetSpeed);
assignin('base','actorProfiles', actorProfiles);

evalin('base', 'helperCreateSensorFusionBusObjects');
end

%% Vehicle dynamics parameters from scenario
% function egoVehDyn = egoVehicleDynamicsParams(ego, egoActor)
% % Ego pose for vehicle dynamics from RoadRunner Scenario.
% egoVehDyn.X0  =  ego.Position(1); % (m)
% egoVehDyn.Y0  = -ego.Position(2); % (m)
% egoVehDyn.VX0 =  ego.Velocity(1); % (m/s)
% egoVehDyn.VY0 = -ego.Velocity(2); % (m/s)
% 
% % Adjust sign and unit of yaw
% egoVehDyn.Yaw0 = -deg2rad(ego.Yaw); % (rad)
% 
% % Longitudinal velocity
% egoVehDyn.VLong0 = hypot(egoVehDyn.VX0,egoVehDyn.VY0); % (m/sec)
% 
% % Distance from center of gravity to axles
% egoVehDyn.CGToFrontAxle = egoActor.Length/2 - egoActor.FrontOverhang;
% egoVehDyn.CGToRearAxle  = egoActor.Length/2 - egoActor.RearOverhang;
% end
