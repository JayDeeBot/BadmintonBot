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
    radii = [0.1,0.2,0.3];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    ellipsoidEndEffector_h = surf(X,Y,Z,'FaceAlpha',0.1); % Make the ellipsoid translucent (so we can see the inside and outside points)
    drawnow();

    % Test if any vertices of the rectangular prism are inside the ellipsoid
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
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
    centerPoint = [xlink1 ylink1 zlink1];
    radii = [0.1,0.2,0.3];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    % Test if any vertices of the rectangular prism are inside the
    % ellipsoid
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end

    xlink2 = linkPose2(1,4);
    ylink2 = linkPose2(2,4);
    zlink2 = linkPose2(3,4);

    % Plot Ellipsoid Around Robot Link2
    centerPoint = [xlink2 ylink2 zlink2];
    radii = [0.1,0.2,0.3];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % ellipsoidLink2_h = surf(X,Y,Z,'FaceAlpha',0.1);
    % 
    % drawnow();
    % Make the ellipsoid translucent (so we can see the inside and outside points)
    % alpha(0.1);

    % Test if any vertices of the rectangular prism are inside the
    % ellipsoid
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end

    xlink3 = linkPose3(1,4);
    ylink3 = linkPose3(2,4);
    zlink3 = linkPose3(3,4);

    % Create Ellipsoid Around Robot End Link 3
    centerPoint = [xlink3 ylink3 zlink3];
    radii = [0.1,0.2,0.3];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    % Test if any vertices of the rectangular prism are inside the
    % ellipsoid
    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end

    xlink4 = linkPose4(1,4);
    ylink4 = linkPose4(2,4);
    zlink4 = linkPose4(3,4);

    % Plot Ellipsoid Around Robot Link 4
    centerPoint = [xlink4 ylink4 zlink4];
    radii = [0.1,0.2,0.3];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end
    
    xlink5 = linkPose5(1,4);
    ylink5 = linkPose5(2,4);
    zlink5 = linkPose5(3,4);

    % Create Ellipsoid Around Robot Link 5
    centerPoint = [xlink5 ylink5 zlink5];
    radii = [0.1,0.2,0.3];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end    

    xlink6 = linkPose6(1,4);
    ylink6 = linkPose6(2,4);
    zlink6 = linkPose6(3,4);

    % Create Ellipsoid Around Robot Link 6
    centerPoint = [xlink6 ylink6 zlink6];
    radii = [0.1,0.2,0.3];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    algebraicDist = GetAlgebraicDist(points, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);

    if pointsInside > 0
        check = 1;
        logMessage = sprintf('There are ', num2str(size(pointsInside,1)),' points inside the ellipsoid');
    end

    try delete(ellipsoidEndEffector_h); end


end