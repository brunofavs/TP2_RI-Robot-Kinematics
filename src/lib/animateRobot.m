function animateRobot(H, AAA, P, h, sd, plotpath, robot)

    for k = 1:size(AAA, 4) %For every frame
        org = linkOrigins(AAA(:, :, :, k)); %Get the links origins

        %Update the line linking the links

        h.XData = org(1, :);
        h.YData = org(2, :);
        h.ZData = org(3, :);

        %Calculate the transformation for each link based on the previous links
        T = eye(4);

        for j = 1:size(AAA, 3)
            T = T * AAA(:, :, j, k);
            Pk = T * P;
            set(H(j), 'Vertices', Pk(1:3, :)'); %Update the H(j) patch
        end



        AAc(:,:,1) = AAA(:,:,1,k);

        for d=2:size(AAA,3)
            AAc(:,:,d) = AAc(:,:,d-1)*AAA(:,:,d,k);
        end

        % * Joint A
        P_joints = AAc(:,:,1) * robot.homogenous_vertices.jointA;
        set(robot.handlers.jointA, 'Vertices', P_joints(1:3, :)')

        % * Joint B
        P_joints = AAc(:,:,2) * robot.homogenous_vertices.jointB;
        set(robot.handlers.jointB, 'Vertices', P_joints(1:3, :)')

        % * Joint C
        P_joints = AAc(:,:,3) *  robot.homogenous_vertices.jointC;
        set(robot.handlers.jointC, 'Vertices', P_joints(1:3, :)')
        
        % * Joint D
        P_joints = AAc(:,:,4) * trans(50,-100,0)*robot.homogenous_vertices.jointD;
        set(robot.handlers.jointD, 'Vertices', P_joints(1:3, :)')

        % * Joint E
        P_joints = AAc(:,:,5) *trans(0,-100,0)* robot.homogenous_vertices.jointE;
        set(robot.handlers.jointE, 'Vertices', P_joints(1:3, :)')

        % * Joint F
        P_joints = AAc(:,:,6) * rotx(pi/2)* robot.homogenous_vertices.jointF;
        set(robot.handlers.jointF, 'Vertices', P_joints(1:3, :)')

        % * Joint Fvar
        P_joints = AAc(:,:,7) * trans(0,-robot.dimensions.dLf_max-150,0)*robot.homogenous_vertices.jointFvar;
        set(robot.handlers.jointFvar, 'Vertices', P_joints(1:3, :)')

        % * Joint G
        P_joints = AAc(:,:,8) *robot.homogenous_vertices.jointG;
        set(robot.handlers.jointG, 'Vertices', P_joints(1:3, :)')

        % * Joint H
        P_joints = AAc(:,:,9) * trans(-robot.dimensions.Lh,-robot.dimensions.Lh/2,-1000)* robot.homogenous_vertices.jointH;
        set(robot.handlers.jointH, 'Vertices', P_joints(1:3, :)')

        % % * Joint x
        % P_joints = AAc(:,:,2) * robot.homogenous_vertices.jointx;
        % set(robot.handlers.jointx, 'Vertices', P_joints(1:3, :)')













            if plotpath
                X = T(1, 4); Y = T(2, 4); Z = T(3, 4);
                plot3(X, Y, Z, '.r', 'MarkerSize', 20);
            end
            
        end
        
        pause(sd)
    end



        