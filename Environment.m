function Environment ()

 %% set up view and lighting

view(3)
light("Style","local","Position",[-10 5 5]); % mood lighting
light("Style","local","Position",[-10 -5 5]);


axis([-3, 3, -3, 3, 0, 3])
hold on 


 %% place Objects features
    PlaceObject('Serving_Table.PLY',[0.6,0.7,0]); % DoBot serving table (H:1.5m L:1m W:0.5m)
    PlaceObject('Robot_Base.PLY',[0,1.5,0]); % Badminton bot base (H:0.5m L:0.6m W:3.6m)
    PlaceObject('Net.PLY',[0,0,0]); % Badminton net (H:1m x W:3m)
    PlaceObject('Exclusion_Fence.PLY',[0,0,0]); % Saftey Exclusion Fence (H:2m L:2m W:4m)
    PlaceObject('Light_Curtain.PLY',[-2,0.15,0]); % Saftey  Light Curtain 1 (H:1.8m L:0.1m W:0.1m)
    PlaceObject('Light_Curtain.PLY',[-2,2.85,0]); % Saftey  Light Curtain 2 (H:1.8m L:0.1m W:0.1m)
    PlaceObject('GUI_E-Stop.PLY',[2,-1,0]); % Player GUI and E- stop (H:1.5m L:0.6m W:0.4m)



    hold on
    
    %place Badminton Court (4m x 4m)
    set(0,'DefaultFigureWindowStyle','docked');
    surf([-2,-2;2,2] ...
    ,[-2,2;-2,2] ...
    ,[0.00,0.00;0.00,0.00] ...
    ,'CData',imread('badminton_court_716_352.jpeg') ...
    ,'FaceColor','texturemap');

    %% place Saftey Feature
    % Create Light Curtain

bottomLeft = [-2,0.15,0.15];
topRight = [-2,2.85,1.5];

laserStartPoint = [];
laserEndPoint = [];
laserNormals = [bottomLeft(1)-topRight(1), bottomLeft(2)-topRight(2), 0];

laserCenters = 0.05;

for i = 0.1 : laserCenters : topRight(3)+0.1
    laserStartPoint = [laserStartPoint; bottomLeft(1), bottomLeft(2), bottomLeft(3)+i];
end

for i = 0.1 : laserCenters : topRight(3)+0.1
    laserEndPoint = [laserEndPoint; topRight(1), topRight(2), bottomLeft(3)+i];
end

numLasers = size(laserStartPoint(:,1));

for i = 1 : numLasers

    % Then plot the start and end point in green and red, respectively.            
    hold on;
    plot3(laserStartPoint(i, 1),laserStartPoint(i, 2),laserStartPoint(i, 3) ,'r*');
    plot3(laserEndPoint(i, 1),laserEndPoint(i, 2),laserEndPoint(i, 3) ,'r*');
    plot3([laserStartPoint(i, 1),laserEndPoint(i, 1)],[laserStartPoint(i, 2),laserEndPoint(i, 2)],[laserStartPoint(i, 3),laserEndPoint(i, 3)] ,'r');
    axis equal

end







