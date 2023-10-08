%% Test script for Assignment 2 - Collision Avoidance
% Simulation of robot avoiding collision with a table in its worspace

close all;
clf
clc
clear

PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0,0,0]); % Place Table

%% Rectangular Prism
% Generate Rectangular Prism Surrounding the table model
side1 = 2.15; % Set Rectangular Prism Sides the same as the table model
side2 = 1.45;
side3 = 0.5;
centerpoint = [0,0,0.25]; % Set centerpoint to make Rectangular Prism surround table model

plotOptions.plotFaces = false; % Set to false to make Rectangular Prism invisible
[vertex,faces,faceNormals] = RectangularPrism(side1, side2, side3, centerpoint, plotOptions);
axis equal
camlight

%% Generate Robot Model

% Make a 3DOF model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);       
robot = SerialLink([L1 L2 L3],'name','myRobot'); 
robot.base = transl(-1.5, -1.5, 0.25);
q = zeros(1,3); % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-4 4 -4 4 -0.05 2]; % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale); % Plot the robot

% Get the transform of every joint (i.e. start and end of every link)
tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;    
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

%%
robot.teach
%%

qWaypoints = [];
q = [0 0 0];

for i = 1 : 3
    for j = 1 : 90
        q(i) = q(i) - 1;
        qWaypoints = [qWaypoints; q];
    end
end

for i = 1 : 22
        q(2) = q(2) - 1;
        qWaypoints = [qWaypoints; q];
end
for i = 1 : 22
        q(3) = q(3) + 1;
        qWaypoints = [qWaypoints; q];
end
for i = 1 : 22
        q(3) = q(3) + 1;
        qWaypoints = [qWaypoints; q];
end



%%

q2 = deg2rad([-90 -90 -90]);
q3 = deg2rad([-90 -112.5 -67.5]);
q4 = deg2rad([-180 -90 0]);
q5 = deg2rad([-180 0 0]);
q6 = deg2rad([-198.5 0 0]);
q7 = deg2rad([-225 0 0]);

wayPoints = [q2 q3 q4 q5 q6 q7];

%% Move Robot without avoiding collisions

firstJoint = pi/2;

for j = 0 : pi/180 : firstJoint

    robot.animate([j 0 0]);
    pause(0.01)

end

%%
q1
q2

robot.animate(q1);

[qMatrix qWaypoints] = CollisionAvoidance(robot, )

robot.animate(qMatrix);

%% Move Robot & Avoid Collisions

start = [0 0 0];
finish = [pi/2 0 0];

traj = jtraj(start,finish,20);

isCollision = true;

while(isCollision)

    if ~IsCollision(robot,traj,faces,vertex,faceNormals)

        for i = 0 : size(traj(:,1))

            robot.animate(traj);
            pause(0.01);

        end

    else

        %pose = robot.fkine(robot.getpos);
        qNow = robot.getpos;
        change = [-pi/10 0 0];
        wayPoint = qNow + change;
        robot.animate(wayPoint);
        traj = jtraj(wayPoint, finish, 20 - i);
        %[intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        isCollision = true;

    end

    if qNow(1) == finish(1) && qNow(2) == finish(2) && qNow(3) == finish(3)

        isCollision = false;

    end

end

%%

q1 = [0,0,0];
q2 = [pi/2,0,0];

[qMatrix,qWaypoints] = CollisionAvoidance(robot,[0 0 0],2.4,q1,q2)

% Randomly select waypoints (primative RRT)
% robot.animate(q1);
% qWaypoints = [q2 q3 q4 q5 q6 q7];
% qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
% if IsCollision(robot,qMatrix,faces,vertex,faceNormals)
%     error('Collision detected!!');
% else
%     display('No collision found');
% end
robot.animate(qMatrix);        

%%
% 3.3: Randomly select waypoints (primative RRT)
robot.animate(q1);
%qWaypoints = [q1;q8];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
            robot.animate(qMatrixJoin);
            size(qMatrix)
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q8],deg2rad(10));
            if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,3) - 1) * pi;
            while IsCollision(robot,qRand,faces,vertex,faceNormals)
                qRand = (2 * rand(1,3) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end
robot.animate(qMatrix)