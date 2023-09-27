classdef RacketBot < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'RacketBot';
    end
    
    methods
%% Constructor
function self = RacketBot(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr  ;
            
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.15,'a',0.1,'alpha',pi/2,'qlim',[-pi pi]);
            link(2) = Link('d',0,'a',0.2,'alpha',0,'qlim',[-pi pi]);
            link(3) = Link('d',0,'a',0.1,'alpha',pi/2,'qlim', deg2rad([-90 90]), 'offset',0);
            link(4) = Link('d',0,'a',0,'alpha',-pi/2,'qlim', deg2rad([-360 360]), 'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
