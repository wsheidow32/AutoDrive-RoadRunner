%% Initialization

clc; clear; close all

project_start;

%% Open Model

warning('off','all')
open("ADC_RoadRunner.slx")
%% Set File Path for RoadRunner Application

rrAppPath = "C:\Program Files\RoadRunner R2024b\bin\win64"; %Roadrunner app path
settings.roadrunner.application.InstallationFolder.TemporaryValue = rrAppPath; %set path for Matlab connection

%% Set Project Path

rrProjectPath = "C:\Users\wshei\OneDrive\Desktop\RodRunner Simulator\Third Project";
rrApp = roadrunner(rrProjectPath); %opening Roadrunner to project folder

%% Open Scenario

ScenarioName = "TrafficLight_LeftTurn.rrscenario";
%load parameter.mat;
load businfo.mat;
load('BusWorldToVehiclcleActors\3\BusWorldToVehicleActors.mat')
openScenario(rrApp,ScenarioName);

%% Set Up Simulation

rrSim = rrApp.createSimulation;
scenario.SimulatorInstance = rrApp;
%scenario.SimulationInstance = rrSim;
rrSim.set('Logging','on');
maxSimulationTimeSec = 10;
STEER_RATIO = -0.0582;
LaneWidth = 3.85;
Ts = 0.05;

set(rrSim,StepSize=Ts)
set(rrSim,MaxSimulationTime=maxSimulationTimeSec)
helperSLAEBWithRRSetup(rrApp,rrSim,scenarioFileName="TrafficLight_LeftTurn")  %need to change based on Roadrunner scenario and behavior to use

%% Run Simulation

addpath('Misc Models');
set(rrSim,SimulationCommand="Start")
while strcmp(rrSim.get("SimulationStatus"),"Running")
  pause(1)
end
%{
%% Create Velocity Metric Plots

% Get logged results from scenario
simLog = get(rrSim,"SimulationLog");

% Counting Actors
actorSim = get(simLog,"ActorIDs");
numActors = length(actorSim) - 1;
 
% Initialize the Cell Array to store the Actor Velocity Results results
velocityActors = cell(1, numActors);
velMagActor = [];

% Loop through each ActorID (from 1 to numActors) to gain graphing metrics
for i = 1:numActors
    % Get logged times and velocities @ time
    velocityActors{i} = get(simLog, "Velocity", "ActorID", i);
    
    % calculates the magnitude of the velocity
    velocityArray = velocityActors{i};
    velMagActor{i} = arrayfun(@(x) norm(x.Velocity, 2), velocityArray);
end

% Set up Time Array for Plotting
timeArray = velocityActors{1};
time = [timeArray.Time];

% Plot each Actor velocities with resepect to simulation time
figure('Name', 'Actor Velocities')
hold on
for i = 1:numActors
    plot(time,velMagActor{i})
    legendSubset{i} = ['Actor ', num2str(i), ' Velocity Magnitude'];
end
grid on
title("Actor Velocities from RoadRunner Scenario")
ylabel("Velocity (m/sec)")
xlabel("Time (sec)")
legend(legendSubset)

%% Route Metric Plotting

% HD map specifications
hdMap = get(rrSim,"Map");
lanes = hdMap.Lanes;

% Lane specifications
figure('Name', 'Route')
hold on
for i = 1:numel(lanes)
    control_points = lanes(i).Geometry;
    x_coordinates = control_points(:,1);
    y_coordinates = control_points(:,2);
    plot(x_coordinates,y_coordinates,"black");
end
axis equal

% Route Specifications
for i = 1:numActors
    % extract positions of the vech
    poseActors{i} = get(simLog, "Pose", "ActorID",1);
    positionActori_x = arrayfun(@(x) x.Pose(1,4),poseActors{i});
    positionActori_y = arrayfun(@(x) x.Pose(2,4),poseActors{i});
    plot(positionActori_x,positionActori_y,"r",LineWidth=2)
end
 
title("Actor Positions from RoadRunner Scenario")
ylabel("Y (m)")
xlabel("X (m)")

figure(gcf)
%}

%% Close Simulation

%close(scenario.SimulatorInstance)
disp("Simulation Complete")