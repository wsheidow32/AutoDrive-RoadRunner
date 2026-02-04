if contains(fileNameWithOutExt, "FourWayLight")

pedAsset = roadrunner.Asset('Four Way Light/Assets/Characters/Citizen_Male.rrchar');
carAsset = roadrunner.Asset('Four Way Light/Assets/Vehicles/Sedan.fbx');

if contains(fileNameWithOutExt, "Pedestrian")
if contains(fileNameWithOutExt, "Close")
if contains(fileNameWithOutExt, "Right")
pedestrianPosition = [8.0, 8.0 * rand(), 0];
pedestrianDegrees = -180;
elseif contains(fileNameWithOutExt, "Left")
pedestrianPosition = [8.0 * rand(), -11.0, 0];
pedestrianDegrees = 90;
elseif contains(fileNameWithOutExt, "Straight")
pedestrianPosition = [8.0, 8.0 * rand(), 0];
pedestrianDegrees = -180;
end
elseif contains(fileNameWithOutExt, "Far")
if contains(fileNameWithOutExt, "Right")
pedestrianPosition = [8.0 * rand(), 8.0, 0];
pedestrianDegrees = 90;
else
pedestrianPosition = [-9.0, 8.0 * rand(), 0];
pedestrianDegrees = -180;
end
end

newPed = addActor(rrSim.Scenario, pedAsset, pedestrianPosition);
setActorOrientation(newPed, pedestrianDegrees);

elseif contains(fileNameWithOutExt, "CarYield")
if contains(fileNameWithOutExt, "Parrallel")
if contains(fileNameWithOutExt, "Left")
carYieldPosition = [-1.7, 99.0 * rand(), 0];
carYieldDegrees = -180;
else % Right or Straight
carYieldPosition = [-99.0 * rand(), -1.7, 0];
carYieldDegrees = -90;
end
elseif contains(fileNameWithOutExt, "Perpendicular")
if contains(fileNameWithOutExt, "Left")
carYieldPosition = [-99.0 * rand(), -1.7, 0];
carYieldDegrees = -90;
else
carYieldPosition = [1.7, -99.0 * rand(), 0];
carYieldDegrees = 0;
end
end

newCar = addActor(rrSim.Scenario, carAsset, carYieldPosition);
setActorOrientation(newCar, carYieldDegrees);
end
else 

pedAsset = 'Four Way Stop/Assets/Characters/Citizen_Male.rrchar';
carAsset = 'Four Way Stop/Assets/Vehicles/Sedan.fbx';

if contains(fileNameWithOutExt, "Pedestrian")
if contains(fileNameWithOutExt, "Close")
if contains(fileNameWithOutExt, "Right")
pedestrianPosition = [8.0, 8.0 * rand(), 0];
pedestrianDegrees = -180;
elseif contains(fileNameWithOutExt, "Left")
pedestrianPosition = [8.0 * rand(), -11.0, 0];
pedestrianDegrees = 90;
elseif contains(fileNameWithOutExt, "Straight")
pedestrianPosition = [8.0, 8.0 * rand(), 0];
pedestrianDegrees = -180;
end
elseif contains(fileNameWithOutExt, "Far")
if contains(fileNameWithOutExt, "Right")
pedestrianPosition = [8.0 * rand(), 8.0, 0];
pedestrianDegrees = 90;
else
pedestrianPosition = [-9.0, 8.0 * rand(), 0];
pedestrianDegrees = -180;
end
end

newPed = addActor(rrSim, pedAsset, pedestrianPosition);
setActorOrientation(newPed, pedestrianDegrees);

elseif contains(fileNameWithOutExt, "CarYield")
if contains(fileNameWithOutExt, "Parrallel")
if contains(fileNameWithOutExt, "Left")
carYieldPosition = [-1.7, 99.0 * rand(), 0];
carYieldDegrees = -180;
else
carYieldPosition = [-99.0 * rand(), -1.7, 0];
carYieldDegrees = -90;
end
elseif contains(fileNameWithOutExt, "Perpendicular")
if contains(fileNameWithOutExt, "Left")
carYieldPosition = [-99.0 * rand(), -1.7, 0];
carYieldDegrees = -90;
else
carYieldPosition = [1.7, -99.0 * rand(), 0];
carYieldDegrees = 0;
end
end

newCar = addActor(rrSim.Scenario, carAsset, carYieldPosition);
setActorOrientation(newCar, carYieldDegrees);

end

end