    nlobj = nlmpc(6,6,'MV',[1 2]);
    nlobj.Ts = 0.05;
    nlobj.PredictionHorizon = 10;
    nlobj.ControlHorizon = 10;
    
    nlobj.Model.StateFcn = 'StateFunctionVehicle';
    nlobj.Jacobian.StateFcn = 'VehicleStateJacFcn';
    
    nlobj.Model.OutputFcn = 'VehicleOutputFcn';
    nlobj.Jacobian.OutputFcn = 'OutputJacobianFcn';
    
    nlobj.MV(1).Min = -3;
    nlobj.MV(1).Max = 3;
    nlobj.MV(2).Min = -0.55;
    nlobj.MV(2).Max = 0.55;
    
    nlobj.MV(1).ScaleFactor = 6;    
    nlobj.MV(2).ScaleFactor = 2.26; 
    nlobj.OV(4).ScaleFactor = 10;


    nlobj.Weights.ManipulatedVariablesRate = [0.3 0.3];
    nlobj.Weights.OutputVariables = [1 1 0.5 1 0.5 0.0];
    
    %x0 = [0 10 1.57 0 0 0];
    %u0 = [0.125 0.4];
    %ref0 = [10 10 1.57 10 0 0];
    %md0 = 0.1;
    %validateFcns(nlobj, x0, u0, [], {}, ref0);
