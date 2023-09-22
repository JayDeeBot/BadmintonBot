clf

view(3);

MagicDobot = DobotMagician(trotz(deg2rad(180)));

% MagicDobot.model.base = ;

q0 = [0 0 0 0 0];

% MagicDobot.model.plot(q0);

% MagicDobot.model.teach();

jointLimits = deg2rad([0 0 0 90 0]);

goaltr = MagicDobot.model.fkine(deg2rad([0 0 0 90 0]));

q1 = wrapToPi(MagicDobot.model.ikcon(goaltr,jointLimits));

traj = jtraj(q0, q1, 50);

planeNormal = [-1,0,0];
planePoint = [1.5,0,0];

hold on

[Y,Z] = meshgrid(0:0.1:0.5,0:0.1:0.5);
X = repmat(0.3,size(Y,1),size(Y,2));
surf(X,Y,Z);

for i = 1:50

    hold on
    try delete(cone_h); end
    MagicDobot.model.animate(traj(i, :));
    endEffectorTr = MagicDobot.model.fkine(MagicDobot.model.getpos()).T;

    [X,Y,Z] = cylinder([0,0.1],6);
    Z = Z * 10;
    updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';
    conePointsSize = size(X);
    hold on
    cone_h = surf(reshape(updatedConePoints(:,1),conePointsSize) ...
     ,reshape(updatedConePoints(:,2),conePointsSize) ...
     ,reshape(updatedConePoints(:,3),conePointsSize));

    coneEnds = [cone_h.XData(2,:)', cone_h.YData(2,:)', cone_h.ZData(2,:)'];

    rayIndex = 1;
    rayStart = endEffectorTr(1:3,4)';
    rayEnd = coneEnds(rayIndex,:);

    [intersectionPoint,check] = LinePlaneIntersection(planeNormal,planePoint,rayStart,rayEnd);

    if check == 1
         pause;
    end

    pause(0.01)

end