classdef HelperPathActionAdapter < matlab.System
    %HelperPathActionAdapter obtains path information from path action.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.
    %
    
    % Copyright 2021 The MathWorks, Inc.
    
    properties(Access = private)
        Path = zeros(500,3);
        NumPoints = 0;
    end
    
    methods(Access = protected)
        function [Path, NumPoints] = stepImpl(obj,PathAction)
            if ~isempty(PathAction)
                obj.Path = PathAction.PathTarget.Path;
                obj.NumPoints = double(PathAction.PathTarget.NumPoints);
            end
            Path = obj.Path;
            NumPoints = obj.NumPoints;
        end
        
        function num = getNumOutputsImpl(~)
            num = 2;
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = [mfilename("class"), newline, newline];
        end
        
        function interface = getInterfaceImpl(~)
            import matlab.system.interface.*;
            interface = [Input("in1", Message), ...
            Output("out1", Data), Output("out2", Data)];
        end

        function [out1, out2] = getOutputSizeImpl(~)
            out1 = [500 3];
            out2 = [1 1];
        end

        function [out1, out2] = getOutputDataTypeImpl(~)
            out1 = "double";
            out2 = "double";
        end

        function [out1, out2] = isOutputComplexImpl(~)
            out1 = false;
            out2 = false;            
        end

        function [out1, out2] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
        end
        
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end
