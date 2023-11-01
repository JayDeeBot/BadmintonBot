clear all
clc
clf
set(0,'DefaultFigureWindowStyle','docked');

Environment;
GUI

%%
bricknum = [xPos,-1.5,0.1];
[f,v,data] = plyread('shuttleCock.PLY','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
BrickVertexCount = size(v,1);
BrickMesh_h = trisurf(f,v(:,1)+bricknum(1,1),v(:,2)+bricknum(1,2), v(:,3)+bricknum(1,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

%r.model.teach(zeros(1,7));
%%
centerpnt = [0.3,0.6,1];
side = 1.2;
plotOptions.plotFaces = false;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

%%
RightCourtWithColliAvoid(r,centerpnt,side,xPos,angle,velocity);
%%
qMatrix = MiddleCourtMode(r,xPos,angle,velocity);
%%
LeftCourt(r,xPos,angle,velocity);
%%
MiddleCourtLow(r,xPos,angle,velocity);
%%
q1C= zeros(1,7);
q2C = [-2 deg2rad([0 0 0 0 0 -30])];
%%
[qMatrix qWayPoints] = CollisionAvoidance(r,centerpnt,side,q1C,q2C);
%% Place the Dobot Magician 

r2 = DobotMagician(transl(0.5,0.9,1.5)*trotz(pi));
%%
%Second shuttlecock
bricknum1 = [0.5,0.6,1.55]; 
[f,v,data] = plyread('shuttleCock.PLY','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
BrickVertexCount = size(v,1);
BrickMesh_h1 = trisurf(f,v(:,1)+bricknum1(1,1),v(:,2)+bricknum1(1,2), v(:,3)+bricknum1(1,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
%%
% One more shuttlecock
bricknum2 = [0.5,0.7,1.55]; 
[f,v,data] = plyread('shuttleCock.PLY','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
BrickVertexCount = size(v,1);
BrickMesh_h2 = trisurf(f,v(:,1)+bricknum2(1,1),v(:,2)+bricknum2(1,2), v(:,3)+bricknum2(1,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
%% PLACE THE SHUTTLECOCK NEXT TO THE DOBOT
brick1 = transl(0.5,0.6,1.6)*trotx(-pi/2); %Shuttlecock pose
UpdatedPoints1 = [brick1 * [v,ones(BrickVertexCount,1)]']';
BrickMesh_h1.Vertices = UpdatedPoints1(:,1:3);

%%
brick2 = transl(0.5,0.7,1.6)*trotx(-pi/2); %Shuttlecock pose
UpdatedPoints2 = [brick2 * [v,ones(BrickVertexCount,1)]']';
BrickMesh_h2.Vertices = UpdatedPoints2(:,1:3);

%%

% Move the DOBOT TO HOME POSITION
defaultRealQ  = [0,pi/4,pi/4,pi/2,0];
r2.model.animate(defaultRealQ);
qNow = r2.model.getpos();
%% DO NOT RUN THIS JUST FOR TESTING
 qNext = r2.model.ikcon(transl(0.5,0.7,1.69),defaultRealQ);
% 
% r2.model.animate(qNext);
% ANIMATE DOBOT TO PICK THE SHUTTLECOCK

qmatrix = jtraj(defaultRealQ,qNext,100);
for i = 1:100
    r2.model.animate(qmatrix(i,:));
    pause(0.01);
end
% BRING THE SHUTTLECOCK TO DROP POSITION
for i = 100:-1:1
    brick1 = r2.model.fkineUTS(qmatrix(i,:));
    BrickPose1 = brick1*transl(0,0,-0.08)*trotx(-pi/2) ;
   
    UpdatedPoints1 = [BrickPose1 * [v,ones(BrickVertexCount,1)]']';
    
    BrickMesh_h2.Vertices = UpdatedPoints1(:,1:3);

    r2.model.animate(qmatrix(i,:));
    pause(0.01);
end
% MOVE RACKETBOT TO HITTING POSE
qHittingPose = [-0.3 deg2rad([30,30,-10,0,-60,0])];
r.model.animate(zeros(1,7));
qmatrix2 = jtraj(zeros(1,7),qHittingPose,50);
for i = 1:50
    r.model.animate(qmatrix2(i,:));
    pause(0.01);
end

% RMRC - ONLY RUN ONCE
steps = 50;
THitting = r.model.fkineUTS(qHittingPose);
x1 = [THitting(1,4) THitting(2,4) THitting(3,4) 0 0 0]';
x2 = [THitting(1,4) THitting(2,4)-0.1 THitting(3,4) 0 0 0]';
deltaT = 0.01;                                        % Discrete time step

xR = zeros(6,steps);

s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    xR(:,i) = x1*(1-s(i)) + s(i)*x2; 

end

qMatrix = [nan(steps,6) zeros(steps,1)];

qMatrix(1,:) = qHittingPose;                 

for i = 1:steps-1
    xdot = (xR(:,i+1) - xR(:,i))/deltaT;                             
    J = r.model.jacob0(qMatrix(i,:));            
    J = J(1:6,1:6);                           
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix(i+1,1:5) =  qMatrix(i,1:5) + deltaT*qdot(1:5,1)';
    qMatrix(i+1,6) = deg2rad(-80)+deg2rad(50)/50*i;
    qMatrix(i+1,7) = 0;
end
%
for i = 1:50
    BrickPose1 = brick1*transl(0,0,-0.013*i)*trotx(pi/100*i) ;
   
    UpdatedPoints1 = [BrickPose1 * [v,ones(BrickVertexCount,1)]']';
    
    BrickMesh_h2.Vertices = UpdatedPoints1(:,1:3);
    r.model.animate(qMatrix(i,:));
    pause(0.0001);
end
%Corrdinate of the shuttlecock now = (0,0.94,1.125)

% Flying trajectory
angle = -50;
velocity = -4;
xPos = 0;
g = 9.81;
theta = deg2rad(angle);
tEnd = 2*velocity*sin(theta)/g;
%t_end = t1_end + t2_end;
t = linspace(0,tEnd+0.32);

y = velocity*t*cos(theta)+0.94;
z = 1.45+velocity*t*sin(theta)-0.5*g*t.^2;
x = xPos + -0.5.*t;


% FLYING shuttlecock
for i=1:100
    brick = transl(x(1,i),y(1,i),z(1,i))*trotx(i*pi/100);

    BrickPose = brick ;

    %MOVE THE BRICK. Update the point then multiply it to the vertices
    UpdatedPoints1 = [BrickPose * [v,ones(BrickVertexCount,1)]']';

    %The vertices are all the rows and 1 to 3 columns of the UpdatedPoints
    BrickMesh_h2.Vertices = UpdatedPoints1(:,1:3);

    pause(tEnd/100);
end

%% Second shuttlecock

