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


        % joint_point_field_names = fieldnames(robot.homogenous_vertices);
        % joint_handlers_field_names = fieldnames(robot.handlers);
        
        total_rot = eye(4);
        % for j = 1:size(AAA, 3)
        for j = 1:7

            if j == 6
                continue
            end
            

            % T =T * AAA(:, :, j, k);
            % if j < 6
            %     P_joints =  T * robot.homogenous_vertices.(joint_point_field_names{j});
            %     % set(robot.handlers.(joint_handlers_field_names{j}),'Vertices', P_joints(1:3,:)')
            % elseif j == 7
            %     P_joints =  T * robot.homogenous_vertices.jointF;
            %     % set(robot.handlers.jointF,'Vertices', P_joints(1:3,:)')

            %     P_joints =  T * robot.homogenous_vertices.jointFvar;
            %     % set(robot.handlers.jointFvar,'Vertices', P_joints(1:3,:)')
                


        end

        if plotpath
            X = T(1, 4); Y = T(2, 4); Z = T(3, 4);
            plot3(X, Y, Z, '.r', 'MarkerSize', 20);

            % disp("X")
            % disp(X)
            % disp("Y")
            % disp(Y)
            % disp("Z")
            % disp(Z)
        end

        pause(sd)
    end

end
