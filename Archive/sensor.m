rrAppPath = "C:\Program Files\RoadRunner R2022b\bin\win64";
rrAppPath = "C:\Program Files\RoadRunner R2022b\bin\win64";
s = settings;
s.roadrunner.application.InstallationFolder.TemporaryValue = rrAppPath;
rrProjectPath = "C:\Users\hunter\Documents\MATLAB\roadrunner_example";
rrApp = roadrunner(rrProjectPath);

load parameter.mat;
openScenario(rrApp,"M_city.rrscenario");
X_o=-232.81;Y_o=125.83;psi_o=-1.185;  %Ego initial position

rrSim = rrApp.createSimulation;
rrSim.set('Logging','on');
Ts = 0.05;
set(rrSim,StepSize=Ts)
helperSLAEBWithRRSetup(rrApp,rrSim,scenarioFileName="M_city")  %need to change based on Roadrunner scenaroi and behavior to use
%% 

set(rrSim,SimulationCommand="Start")
while strcmp(rrSim.get("SimulationStatus"),"Running")
  pause(1)
end