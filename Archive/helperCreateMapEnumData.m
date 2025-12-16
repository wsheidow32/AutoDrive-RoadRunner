%% Enum Classes Used in for RR scenario map information

% NOTE: This is a helper file for example purposes
% and may be removed or modified in the future.

% Copyright 2021 The MathWorks, Inc.

% Define TravelDir
Simulink.defineIntEnumType('TravelDir',{'TRAVEL_DIR_UNSPECIFIED','TRAVEL_DIR_UNDIRECTED','TRAVEL_DIR_FORWARD','TRAVEL_DIR_BACKWARD','TRAVEL_DIR_BIDIRECTIONAL'},[0;1;2;3;4],'Description', 'Lane Travel Direction','DefaultValue', 'TRAVEL_DIR_UNSPECIFIED')

% Define Alignment
Simulink.defineIntEnumType('Alignment',{'ALIGNMENT_UNSPECIFIED','ALIGNMENT_FORWARD','ALIGNMENT_BACKWARD'},[0;1;2],'Description', 'Alignment','DefaultValue', 'ALIGNMENT_UNSPECIFIED')

% Define LaneType
Simulink.defineIntEnumType('LaneType',{'LANE_TYPE_UNSPECIFIED','LANE_TYPE_DRIVING','LANE_TYPE_SHOULDER','LANE_TYPE_BORDER','LANE_TYPE_RESTRICTED','LANE_TYPE_PARKING','LANE_TYPE_CURB','LANE_TYPE_SIDEWALK','LANE_TYPE_CENTER_TURN','LANE_TYPE_BIKING';},[0;1;2;3;4;5;6;7;8;9],'Description', 'Lane type','DefaultValue', 'LANE_TYPE_UNSPECIFIED')
