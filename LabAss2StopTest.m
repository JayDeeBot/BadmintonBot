clf
clc
clear
set(0,'DefaultFigureWindowStyle','docked')		

%% Create 1-link robot
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
robot = SerialLink(L1,'name','myRobot');

q = 0;                                                             
scale = 0.5;
workspace = [-0.5 1.5 -0.5 1.5 -1 1];                                      
robot.plot(q,'workspace',workspace,'scale',scale);                 
hold on;

%% Create sphere
sphereCenter = [1,1,0];
radius = 0.5;
[X,Y,Z] = halfSphere(20);
X = X * radius + sphereCenter(1);
Y = Y * radius + sphereCenter(2);
Z = Z * radius + sphereCenter(3);

%% Plot it
% Triangle mesh plot
tri = delaunay(X,Y,Z);
sphereTri_h = trimesh(tri,X,Y,Z);
        
drawnow();
view(3)
axis equal
            
%% Move Robot
for q = 0:pi/180:pi/2
    robot.animate(q);
    drawnow();
    pause(0.1);
    % Stop the movement when an obstacle is detected
    if checkCollision(robot,sphereCenter,radius) == 1
        robot.fkine(q)
        q
        disp('UNSAFE: Robot stopped')
        break
    end
end

%% Try with Line Plane Intersection

% Extract 3 Points from sphere mesh to create a single triangle to test
% collision with
p1 = [X(1, 1) Y(1, 1) Z(1, 1)];
p2 = [X(2, 1) Y(2, 1) Z(2, 1)];
p3 = [X(3, 1) Y(3, 1) Z(3, 1)];

% Generate normal of trianlge using cross product
n = cross(p2 - p1, p3 -p1);

% Set normal and point for LinePlaneIntersection function
planeNormal = n;
planePoint = p1;

% Then if we have a line (perhaps a robot's link) represented by two points:
% poses = GetLinkPoses(q, robot);
lineStartPoint = [0,0,0];
tr = robot.fkine(q).T;
lineEndPoint = [tr(1, 4), tr(2, 4), tr(3, 4)];

triangleVerts = [X(1:3, 1) Y(1:3, 1) Z(1:3, 1)];

% Then we can use the function to calculate the point of
% intersection between the line (line) and plane (obstacle)
[intersectP,check] = LinePlaneIntersection(planeNormal,planePoint,lineStartPoint,lineEndPoint);
% Use IsIntersectionInsideTriangle to ensure LinePlaneIntersection is
% within the triangle extent
result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts);

%% Move Robot

check = zeros(18*21);
result = zeros(18*21);
ip = zeros(18*21, 3);

for q = 0:pi/180:pi/2
    robot.animate(q);
    drawnow();
    %pause(0.1);

    for i = 1:18

        for j = 1:21

            % Extract 3 Points from sphere mesh to create a single triangle to test
            % collision with
            p1 = [X(i, j) Y(i, j) Z(1, j)];
            p2 = [X(i + 1, j) Y(i + 1, j) Z(i + 1, j)];
            p3 = [X(i + 2, j) Y(i + 2, j) Z(i + 2, j)];
            
            % Generate normal of trianlge using cross product
            n = cross(p1, p2);
            
            % Set normal and point for LinePlaneIntersection function
            planeNormal = n;
            planePoint = p1;
            
            % Then if we have a line (perhaps a robot's link) represented by two points:
            % poses = GetLinkPoses(q, robot);
            lineStartPoint = [0,0,0];
            tr = robot.fkineUTS(q);
            lineEndPoint = [tr(1, 4), tr(2, 4), tr(3, 4)];
            
            triangleVerts = [X(i:i + 2, j) Y(i:i + 2, j) Z(i:i + 2, j)];
            
            % Then we can use the function to calculate the point of
            % intersection between the line (line) and plane (obstacle)
            [intersectP,check(i*j)] = LinePlaneIntersection(planeNormal,planePoint,lineStartPoint,lineEndPoint);
            ip(i*j, :) = intersectP;
            % Use IsIntersectionInsideTriangle to ensure LinePlaneIntersection is
            % within the triangle extent
            result(i*j) = IsIntersectionPointInsideTriangle(intersectP,triangleVerts);

            % Stop the movement when an obstacle is detected
            if result(i*j) == 1 && check(i*j) == 1 || checkCollision(robot,sphereCenter,radius) == 1
                disp('UNSAFE: Robot stopped')
                break
            end

        end

    end

end

