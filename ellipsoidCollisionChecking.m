function [check logMessage] = ellipsoidCollisionChecking(robot, points)

    check = 0;
    logMessage = sprintf('There are zero points inside the ellipsoid');

    % Get Forward Kinematics for Racket Bot 
    tr = robot.model.fkine(robot.model.getpos()).T;
    x = tr(1,4);
    y = tr(2,4);
    z = tr(3,4);

    % Plot Ellipsoid Around Robot End Effector
    centerPoint = [x y z];
    radii = [0.2,0.3,0.15];
    [XEE,YEE,ZEE] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    ellipsoidEndEffector_h = surf(XEE,YEE,ZEE,'FaceAlpha',0.1,'EdgeColor','flat'); % Make the ellipsoid translucent (so we can see the inside and outside points)
    drawnow();

    % Test if any vertices of the rectangular prism are inside the ellipsoid
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf(num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end
    
    [proxy, linkPoses] = robot.model.fkine(robot.model.getpos());
    linkPose1 = linkPoses(1).T;
    linkPose2 = linkPoses(2).T;
    linkPose3 = linkPoses(3).T;
    linkPose4 = linkPoses(4).T;
    linkPose5 = linkPoses(5).T;
    linkPose6 = linkPoses(6).T;

    xlink1 = linkPose1(1,4);
    ylink1 = linkPose1(2,4);
    zlink1 = linkPose1(3,4);

    % Create Ellipsoid Around Robot Link 1
    % centerPoint = [xlink1 ylink1 zlink1];
    % radii = [0.5,0.4,0.1];
    % [X1,Y1,Z1] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % surf(X1,Y1,Z1,'FaceAlpha',0.01,'EdgeColor','none'); % Make the ellipsoid translucent (so we can see the inside and outside points)
    % drawnow();
    % Test if any vertices of the rectangular prism are inside the
    % ellipsoid
    % algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    % pointsInside = find(algebraicDist < 1);
    % 
    % if pointsInside > 0
    %     check = 1;
    %     logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
    % end
    % 
    xlink2 = linkPose2(1,4);
    ylink2 = linkPose2(2,4);
    zlink2 = linkPose2(3,4);

    % Plot Ellipsoid Around Robot Link2
    centerPoint = [xlink2 ylink2 zlink2];
    radii = [0.5,0.4,0.35];
    % [X2,Y2,Z2] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % ellipsoidLink2 = surf(X2,Y2,Z2,'FaceAlpha',0.05,'EdgeColor','interp');
    % drawnow();
    % Test if any vertices of the rectangular prism are inside the
    % ellipsoid
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf(num2str(size(pointsInside,1)),' points inside the ellipsoid of Link2');
    end

    xlink3 = linkPose3(1,4);
    ylink3 = linkPose3(2,4);
    zlink3 = linkPose3(3,4);

    % Create Ellipsoid Around Robot End Link 3
    centerPoint = [xlink3+0.15 ylink3 zlink3];
    radii = [0.2,0.2,0.5];
    % [X3,Y3,Z3] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % surf(X3,Y3,Z3,'FaceAlpha',0.1,'EdgeColor','none');
    % drawnow();
    % Test if any vertices of the rectangular prism are inside the
    % ellipsoid
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf(num2str(size(pointsInside,1)),' points inside the ellipsoid of Link3');
    end

    xlink4 = linkPose4(1,4);
    ylink4 = linkPose4(2,4);
    zlink4 = linkPose4(3,4);

    % Plot Ellipsoid Around Robot Link 4
    centerPoint = [xlink4+0.15 ylink4 zlink4];
    radii = [0.2,0.4,0.2];
    % [X4,Y4,Z4] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % surf(X4,Y4,Z4,'FaceAlpha',0.1,'EdgeColor','none');
    % drawnow();
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf(num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end
    
    xlink5 = linkPose5(1,4);
    ylink5 = linkPose5(2,4);
    zlink5 = linkPose5(3,4);

    % Create Ellipsoid Around Robot Link 5
    centerPoint = [xlink5 ylink5 zlink5];
    radii = [0.3,0.4,0.2];
    % [X5,Y5,Z5] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % ellipsoidLink5 = surf(X5,Y5,Z5,'FaceAlpha',0.1,'EdgeColor','none');
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);
    % drawnow();
    if pointsInside > 0
        check = 1;
        logMessage = sprintf(num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end    

    xlink6 = linkPose6(1,4);
    ylink6 = linkPose6(2,4);
    zlink6 = linkPose6(3,4);

    % Create Ellipsoid Around Robot Link 6
    centerPoint = [xlink6 ylink6 zlink6];
    radii = [0.3,0.4,0.2];
    % [X6,Y6,Z6] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % ellipsoidLink6 = surf(X6,Y6,Z6,'FaceAlpha',0.1,'EdgeColor','none');
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);
    % drawnow();
    if pointsInside > 0
        check = 1;
        logMessage = sprintf(num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end

    try delete(ellipsoidEndEffector_h); end
    %try delete(ellipsoidLink2); end
    % try delete(ellipsoidLink3); end
    % try delete(ellipsoidLink4); end
    % try delete(ellipsoidLink5); end
    % try delete(ellipsoidLink6); end


end