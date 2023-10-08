%% Create Light Curtain
clf
clc
clear

bottomLeft = [-0.5 -0.5 0];
topRight = [-0.5 0.5 1];

laserStartPoint = [];
laserEndPoint = [];
laserNormals = [bottomLeft(1)-topRight(1), bottomLeft(2)-topRight(2), 0];

laserCenters = 0.05;

for i = 0.1 : laserCenters : 1.01
    laserStartPoint = [laserStartPoint; bottomLeft(1), bottomLeft(2), bottomLeft(3)+i];
end

for i = 0.1 : laserCenters : 1.01
    laserEndPoint = [laserEndPoint; topRight(1), topRight(2), bottomLeft(3)+i];
end

numLasers = size(laserStartPoint(:,1));

%% Plot Light Curtain
view(3);
axis([-2 2 -2 2 -2 2]);
for i = 1 : numLasers

    % Then plot the start and end point in green and red, respectively.            
    hold on;
    plot3(laserStartPoint(i, 1),laserStartPoint(i, 2),laserStartPoint(i, 3) ,'r*');
    plot3(laserEndPoint(i, 1),laserEndPoint(i, 2),laserEndPoint(i, 3) ,'r*');
    plot3([laserStartPoint(i, 1),laserEndPoint(i, 1)],[laserStartPoint(i, 2),laserEndPoint(i, 2)],[laserStartPoint(i, 3),laserEndPoint(i, 3)] ,'r');
    axis equal

end

%% Place Cat
kittyCenterpoint = [0 0 0];
[faces,vertex,data] = plyread('Cat.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
kittyVertexCount = size(vertex,1);
kittyMesh_h = trisurf(faces,vertex(:,1)+kittyCenterpoint(1,1),vertex(:,2)+kittyCenterpoint(1,2), vertex(:,3)+kittyCenterpoint(1,3) ...
,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
light('style', 'local', 'Position', [-2 1 1]);
axis([-1 1 -1 1 0 1]);

%% Move Cat & Rectangular Prism
stop = 0;

for i = 0.01 :0.005 : 0.5
    hold on
    kittyPose = transl(-i, 0, 0);
    kittyVertexCount = size(vertex,1);
    UpdatedPoints = [kittyPose * [vertex,ones(kittyVertexCount,1)]']';
    vertex = UpdatedPoints(:,1:3);
    kittyMesh_h.Vertices = UpdatedPoints(:,1:3);
    pause(1)
    for i = 1 : 450
        if  vertex(i, 1) < bottomLeft(1)
            stop = 1;           
        end
    end
    if stop
        stopMessage = sprintf('STOP: Something has crossed the light curtain.');
        disp(stopMessage) % display status to command window for log
        stopMessage_text = text(0, 1, 1, stopMessage); % display status in the figure
        break
    end
end

 %% Has the Cat Crossed the Light Curtain
% 
% for i = 1 : 450
%     if  vertex(i, 1) < bottomLeft(1)
%         pause
%     end
% end
% 
% %% Line Plane Intersection
% 
% intersectionPoints = [];
% check = [];
% catNormals = [];
% isIntersectionInsideTringle = [];
% 
% for i = 1 : 449
% 
%     catNormals = [catNormals; cross(vertex(i, :),vertex(i + 1, :))];
% 
% end
% 
% for i = 1 : 449
% 
%     for j = 1 : 19
% 
%         [intersectionPoints(i, :),check(i)] = LinePlaneIntersection(vertex(i, :),catNormals(i, :),laserStartPoint(j, :),laserEndPoint(j, :));
%         isIntersectionInsideTringle(i) = IsIntersectionPointInsideTriangle(intersectionPoint(i, :),vertex(i, :));
% 
%     end
% 
% end


            