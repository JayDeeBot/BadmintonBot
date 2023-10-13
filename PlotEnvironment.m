function PlotEnvironment()

% workspace = [-2 1.5 -1.5 1.5 -0.01 2];
h11 = PlaceObject('BarrierThick.ply',[0.5,2.5,0]);
hold on;
h12 = PlaceObject('BarrierThick.ply',[-0.5,2.5,0]);
hold on;
h21 =PlaceObject('BarrierThick.ply',[0.5,-2.5,0]);
hold on;
h22 =PlaceObject('BarrierThick.ply',[-0.5,-2.5,0]);
hold on;

% %Put fireExtingguisher, worker, table and emergencyButton
% PlaceObject('fireExtinguisher.ply',[0.85,1.4,0]);
% PlaceObject('Worker.ply',[0,2.3,0.01]);
% PlaceObject('Table.ply',[1,2.5,0]);
% PlaceObject('emergencyStopButton.ply',[1,2.3,0.55]);
% 
% %Plot the barrier and rotate it 
% h31 =PlaceObject('BarrierThick.ply',[-0.5,1.3,0]);
% verts = [get(h31,'Vertices'), ones(size(get(h31,'Vertices'),1),1)] * trotz(pi/2);
% set(h31,'Vertices',verts(:,1:3))
% hold on;
% 
% h32 =PlaceObject('BarrierThick.ply',[0.5,1.3,0]);
% verts = [get(h32,'Vertices'), ones(size(get(h32,'Vertices'),1),1)] * trotz(pi/2);
% set(h32,'Vertices',verts(:,1:3))
% hold on;
% 
% h41 =PlaceObject('BarrierThick.ply',[-0.5,-1.3,0]);
% verts = [get(h41,'Vertices'), ones(size(get(h41,'Vertices'),1),1)] * trotz(pi/2);
% set(h41,'Vertices',verts(:,1:3))
% hold on;
% 
% h42 =PlaceObject('BarrierThick.ply',[0.5,-1.3,0]);
% verts = [get(h42,'Vertices'), ones(size(get(h42,'Vertices'),1),1)] * trotz(pi/2);
% set(h42,'Vertices',verts(:,1:3))
% hold on;

%Plot the concrete ground
set(0,'DefaultFigureWindowStyle','docked');
surf([-2,-2;2,2] ...
    ,[-2,2;-2,2] ...
    ,[0.01,0.01;0.01,0.01] ...
    ,'CData',imread('Court.jpg') ...
    ,'FaceColor','texturemap');

% %Plot the surrounding wall
% surf([-2,-2;-2,-2],[-2,-2;3,3],[0.01,3;0.01,3],'CData',imread('Environment.jpg'),'FaceColor','texturemap');
% surf([-2,-2;2,2],[3,3;3,3],[0.01,3;0.01,3],'CData',imread('Environment.jpg'),'FaceColor','texturemap');
% %Plot NO SMOKING sign
% surf([-2,-2;-2,-2],[-2,-2;-1,-1],[1,2.5;1,2.5],'CData',imread('NoSmoking.png'),'FaceColor','texturemap');
% %Plot Warning sign
% s0 = surf([-2,-2;-2,-2],[-1,-1;0.5,0.5],[-1.5,-0.5;-1.5,-0.5],'CData',imread('EyeSafetyCaution.jpg'),'FaceColor','texturemap');
% rotate(s0, [-0.5 0 0], 90);
% %Plot eye safety caution sign 
% s1 = surf([-0.5,-0.5;0.5,0.5],[3,3;3,3],[-1,0;-1,0],'CData',imread('Robot.jpg'),'FaceColor','texturemap');
% rotate(s1, [0 -1 0], 270);
end