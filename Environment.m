function Environment ()
clf
clc

view(3)
light("Style","local","Position",[-10 5 5]); % mood lighting
light("Style","local","Position",[-10 -5 5]);


axis([-3, 3, -3, 3, 0, 3])
hold on 




    %place Objects features
    PlaceObject('Serving_Table.PLY',[2.2,1.85,0]); % Control table (H:1.5m L:1m W:0.5m)
    PlaceObject('Net.PLY',[0,0,0]); % Badminton net (H:1m x W:3m)
    PlaceObject('Exclusion_Fence.PLY',[0,0,0]); % Saftey Exclusion Fence (H:2m L:2m W:4m)
    PlaceObject('Light_Curtain.PLY',[-2,0.15,0]); % Saftey  Light Curtain 1 (H:1.8m L:0.1m W:0.1m)
    PlaceObject('Light_Curtain.PLY',[-2,1.85,0]); % Saftey  Light Curtain 2 (H:1.8m L:0.1m W:0.1m)
    PlaceObject('GUI_E-Stop.PLY',[2,-1,0]); % Player GUI and E- stop (H:1.5m L:0.6m W:0.4m)

    hold on
    
    %place Badminton Court (4m x 4m)
    set(0,'DefaultFigureWindowStyle','docked');
    surf([-2,-2;2,2] ...
    ,[-2,2;-2,2] ...
    ,[0.00,0.00;0.00,0.00] ...
    ,'CData',imread('badminton_court_716_352.jpeg') ...
    ,'FaceColor','texturemap');


