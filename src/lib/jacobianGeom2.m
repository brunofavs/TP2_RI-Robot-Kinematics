function JJ = jacobianGeom2(DH, MQ, jTyp)

    MDH = generateMultiDH2(DH, MQ, jTyp);

    for k = 1 : size(MDH,3) % Movement Iteration (each DH)
        AAc = accTlinks(MDH(:,:,k));
        
        % org = LinkOrigins(AAc);
        Org = [[0;0;0] squeeze(AAc(1:3,4,:))];
        zis = [[0;0;1] squeeze(AAc(1:3,3,:))];
        ON = Org(:,end);

        for j = 1:size(AAc,3)        
            if (jTyp(j) == 0) % Rotational
                jv = cross(zis(:,j),ON - Org(:,j));
                jw = zis(:,j);
            else              % Linear
                jv = zis(:,j);
                jw = [0 0 0]';
            end

            J(:,j) = [jv; jw];
        end

        JJ(:,:,k) = J;
    end
end