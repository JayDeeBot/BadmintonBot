function [qMatrix,qWaypoints] = CollisionAvoidance(robot,centerPoint,side,q1,q2)
qCurrent = robot.model.getpos();
T = robot.model.fkine(qCurrent).T;
T2 = robot.model.fkine(q2).T;
lowerLeftCorner = [centerPoint(1,1)+side/2,centerPoint(1,2)+side/2];
lowerRightCorner = [centerPoint(1,1)-side/2,centerPoint(1,2)+side/2];
xShift = lowerRightCorner(1,1)-0.1;
yShift = lowerRightCorner(1,2)+0.1;
%if T(1,4) < centerPoint(1,2)

    qWaypoints = [q1 ; robot.model.ikcon(transl([xShift,yShift,T(3,4)]),q1)];
    for i = 1:(floor(side/0.1)+1)
        qWaypoints = [qWaypoints; robot.model.ikcon(transl([xShift,yShift,T(3,4)]),qWaypoints(end,:))];
        xShift = xShift + 0.15;
    end
    qWaypoints = [qWaypoints; q2];
    qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
%end

% if T(1,4) > centerPoint(1,2)
%     qWaypoints = [q1 ; robot.model.ikcon(transl([lowerRightCorner(1,1)+0.1,lowerRightCorner(1,2)+0.1,T(3,4)]),q1)];
%     for i = 1:(floor(side/0.1)+1)
%         qWaypoints = [qWaypoints; robot.model.ikcon(transl([lowerRightCorner(1,1)+0.1,lowerRightCorner(1,2)-0.1*i,T(3,4)]),qWaypoints(end,:))];
% 
%     end
%     qWaypoints = [qWaypoints; q2];
%     qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
% end

end