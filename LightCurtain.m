%% Create Light Curtain


bottomLeft = [-2 3 0];
topRight = [-2 0 1.7];

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
kittyCenterpoint = [-3 1 0];
[faces,vertex,data] = plyread('Cat.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
kittyVertexCount = size(vertex,1);
kittyMesh_h = trisurf(faces,vertex(:,1)+kittyCenterpoint(1,1),vertex(:,2)+kittyCenterpoint(1,2), vertex(:,3)+kittyCenterpoint(1,3) ...
,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
%light('style', 'local', 'Position', [-2 1 1]);

save("vertex.mat","vertex");

%% Move Cat & Rectangular Prism
stop = 0;
kittyVertexCount = size(vertex,1);
for i = -3 :0.01 : -1
    %hold on
    kittyPose = transl(i, 1, 0)*trotz(pi);    
    UpdatedPoints = [kittyPose * [vertex,ones(kittyVertexCount,1)]']'; 
    kittyMesh_h.Vertices = UpdatedPoints(:,1:3);
     
    for i = 1 : 450
        if  kittyMesh_h.Vertices(i,1) > bottomLeft(1)
            stop = 1;           
        end
    end
    if stop
        stopMessage = sprintf('STOP: Something has crossed the light curtain.');
        disp(stopMessage) % display status to command window for log
        stopMessage_text = text(0, 1, 1, stopMessage); % display status in the figure
        break
    end
    pause(1);
end


%% LINE PLANE INTERSECTION
[Y,Z] = meshgrid(0:0.1:3,0:0.1:1.8);
X = repmat(-2,size(Y,1),size(Y,2));
s = surf(X,Y,Z,'FaceAlpha',0,'EdgeColor','r');
planeNormal = [-5,1,0];
planePoint = [-2,1,0];
checkResult = zeros(100,1);
% Then if we have a line (perhaps a robot's link) represented by two points:
%%
steps = 0.016;
for i = 1:50
lineStartPoint = [-3.25+steps*i,1,0];
lineEndPoint = [-2.75+steps*i,1,0];
lineMidPoint = [-3+steps*i,1,0];
kittyPose = transl(lineMidPoint)*trotz(pi);
UpdatedPoints = [kittyPose * [vertex,ones(kittyVertexCount,1)]']';
kittyMesh_h.Vertices = UpdatedPoints(:,1:3);
% Then we can use the function to calculate the point of
% intersection between the line (line) and plane (obstacle)
[intersectionPoints,check] = LinePlaneIntersection(planeNormal,planePoint,lineStartPoint,lineEndPoint);

% The returned values and their means are as follows:
% (1) intersectionPoints, which shows the xyz point where the line


% (2) check intersects the plane check, which is defined as follows:
checkResult(i,1) = check;
if check == 1
    disp("COLLISION HAPPEN!")
end

h1 = plot3(lineStartPoint(1),lineStartPoint(2),lineStartPoint(3) ,'g',"MarkerSize",2);
h2 = plot3(lineEndPoint(1),lineEndPoint(2),lineEndPoint(3) ,'r',"MarkerSize",2);
h3 = plot3([lineStartPoint(1),lineEndPoint(1)],[lineStartPoint(2),lineEndPoint(2)],[lineStartPoint(3),lineEndPoint(3)] ,'k');
%h4 = plot3(intersectionPoints(1),intersectionPoints(2),intersectionPoints(3) ,'k*','MarkerSize',20);
pause(0.01);
delete(h1);
delete(h2);
delete(h3);

end






            