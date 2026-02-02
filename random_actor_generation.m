if contains(fileNameWithOutExt, "Pedestrian"); % Filename has the word Pedestrian
    if contains(fileNameWithOutExt, "Close"); % Filename has the word Close
        if contains(fileNameWithOutExt, "Right"); % Filename has the word Right
        pedestrian = getAsset(rrSim.Project, 'Assets/Characters/Citizen_Male.fbx');
        y1 = 0.00;
        y2 = 8.00;
        yValue = y1 + (y2 - y1) * rand();
        pedestrianPosition = [8.0, yValue ,0]; % x,y,z for pedestrian
        pedestrianDegrees = -180; % Initial orientation
        addPedestrian = addActor(rrSim.Scenario, pedestrian, pedestrianPosition);
        setActorOrientation(addPedestrian, pedestrianDegrees);
        elseif contains(fileNameWithOutExt, "Left"); % Filename has the word Left
        pedestrian = getAsset(rrSim.Project, 'Assets/Characters/Citizen_Male.fbx');
        x1 = 0.00;
        x2 = 8.00;
        xValue = x1 + (x2 - x1) * rand();
        pedestrianPosition = [xValue, -11.0, 0]; % x,y,z for pedestrian
        pedestrianDegrees = 90; % Initial orientation
        addPedestrian = addActor(rrSim.Scenario, pedestrian, pedestrianPosition);
        setActorOrientation(addPedestrian, pedestrianDegrees);
        elseif contains(fileNameWithOutExt, "Straight"); % Filename has the word Straight
        pedestrian = getAsset(rrSim.Project, 'Assets/Characters/Citizen_Male.fbx');
        y1 = 0.00;
        y2 = 8.00;
        yValue = y1 + (y2 - y1) * rand();
        pedestrianPosition = [8.0, yValue ,0]; % x,y,z for pedestrian
        pedestrianDegrees = -180; % Initial orientation
        addPedestrian = addActor(rrSim.Scenario, pedestrian, pedestrianPosition);
        setActorOrientation(addPedestrian, pedestrianDegrees);
        end
    elseif contains(fileNameWithOutExt, "Far"); % Filename has the word Far
        if contains(fileNameWithOutExt, "Right"); % Filename has the word Right
        pedestrian = getAsset(rrSim.Project, 'Assets/Characters/Citizen_Male.fbx');
        x1 = 0.00;
        x2 = 8.00;
        xValue = x1 + (x2 - x1) * rand();
        pedestrianPosition = [xValue, 8.0, 0]; % x,y,z for pedestrian
        pedestrianDegrees = 90; % Initial orientation
        addPedestrian = addActor(rrSim.Scenario, pedestrian, pedestrianPosition);
        setActorOrientation(addPedestrian, pedestrianDegrees);
        elseif contains(fileNameWithOutExt, "Left"); % Filename has the word Left
        pedestrian = getAsset(rrSim.Project, 'Assets/Characters/Citizen_Male.fbx');
        y1 = 0.00;
        y2 = 8.00;
        yValue = y1 + (y2 - y1) * rand();
        pedestrianPosition = [-9.0, yValue , 0]; % x,y,z for pedestrian
        pedestrianDegrees = -180; % Initial orientation
        addPedestrian = addActor(rrSim.Scenario, pedestrian, pedestrianPosition);
        setActorOrientation(addPedestrian, pedestrianDegrees);
        elseif contains(fileNameWithOutExt, "Straight"); % Filename has the word Straight
        pedestrian = getAsset(rrSim.Project, 'Assets/Characters/Citizen_Male.fbx');
        y1 = 0.00;
        y2 = 8.00;
        yValue = y1 + (y2 - y1) * rand();
        pedestrianPosition = [-9.0, yValue , 0]; % x,y,z for pedestrian
        pedestrianDegrees = -180; % Initial orientation
        addPedestrian = addActor(rrSim.Scenario, pedestrian, pedestrianPosition);
        setActorOrientation(addPedestrian, pedestrianDegrees);
        end
    end

elseif contains(fileNameWithOutExt, "CarYield"); % Filename has the word CarYield
    if contains(fileNameWithOutExt, "Parrallel"); % Filename has the word Parrallel
        if contains(fileNameWithOutExt, "Left"); % Filename has the word Left
        carYield = getAsset(rrSim.Project, 'Assets/Vehicles/Sedan.fbx');
        y1 = 0.00;
        y2 = 99.00;
        yValue = y1 + (y2 - y1) * rand();
        carYieldPosition = [-1.7, yValue, 0]; % x,y,z for car yield
        carYieldDegrees = -180; % Initial orientation for car yield
        addCarYield = addActor(rrSim.Scenario, carYield, carYieldPosition);
        setActorOrientation(addCarYield, carYieldDegrees);
        elseif contains(fileNameWithOutExt, "Right"); % Filename has the word Right
        carYield = getAsset(rrSim.Project, 'Assets/Vehicles/Sedan.fbx');
        x1 = 0.00;
        x2 = -99.00;
        xValue = x1 + (x2 - x1) * rand();
        carYieldPosition = [xValue, -1.7, 0]; % x,y,z for car yield
        carYieldDegrees = -90; % Initial orientation for car yield
        addCarYield = addActor(rrSim.Scenario, carYield, carYieldPosition);
        setActorOrientation(addCarYield, carYieldDegrees);
        elseif contains(fileNameWithOutExt, "Straight"); % Filename has the word Straight
        carYield = getAsset(rrSim.Project, 'Assets/Vehicles/Sedan.fbx');
        x1 = 0.00;
        x2 = -99.00;
        xValue = x1 + (x2 - x1) * rand();
        carYieldPosition = [xValue, -1.7, 0]; % x,y,z for car yield
        carYieldDegrees = -90; % Initial orientation for car yield
        addCarYield = addActor(rrSim.Scenario, carYield, carYieldPosition);
        setActorOrientation(addCarYield, carYieldDegrees);
        end
    elseif contains(fileNameWithOutExt, "Perpendicular"); % Filename has the word Perpendicular
        if contains(fileNameWithOutExt, "Left"); % Filename has the word Left
        carYield = getAsset(rrSim.Project, 'Assets/Vehicles/Sedan.fbx');
        x1 = 0.00;
        x2 = -99.00;
        xValue = x1 + (x2 - x1) * rand();
        carYieldPosition = [xValue, -1.7, 0]; % x,y,z for car yield
        carYieldDegrees = -90; % Initial orientation for car yield
        addCarYield = addActor(rrSim.Scenario, carYield, carYieldPosition);
        setActorOrientation(addCarYield, carYieldDegrees);
        elseif contains(fileNameWithOutExt, "Right"); % Filename has the word Right
        carYield = getAsset(rrSim.Project, 'Assets/Vehicles/Sedan.fbx');
        y1 = 0.00;
        y2 = -99.00;
        yValue = y1 + (y2 - y1) * rand();
        carYieldPosition = [1.7, yValue, 0]; % x,y,z for car yield
        carYieldDegrees = 0; % Initial orientation for car yield
        addCarYield = addActor(rrSim.Scenario, carYield, carYieldPosition);
        setActorOrientation(addCarYield, carYieldDegrees);
        elseif contains(fileNameWithOutExt, "Straight"); % Filename has the word Straight
        carYield = getAsset(rrSim.Project, 'Assets/Vehicles/Sedan.fbx');
        y1 = 0.00;
        y2 = -99.00;
        yValue = y1 + (y2 - y1) * rand();
        carYieldPosition = [1.7, yValue, 0]; % x,y,z for car yield
        carYieldDegrees = 0; % Initial orientation for car yield
        addCarYield = addActor(rrSim.Scenario, carYield, carYieldPosition);
        setActorOrientation(addCarYield, carYieldDegrees);
        end
    end
end