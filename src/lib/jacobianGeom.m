function JJ = jacobianGeom(DH,MQ,jTypes)

    addpath('./lib')

MDH = generateMultiDH2(DH, MQ, jTypes);


for i = 1:size(MDH,3)
    origins = [0,0,0]';
    z = [0,0,1]';

    acc = accTlinks(MDH(:,:,i))


    % Iterate over joints
    for j = 1:size(MDH,1)

        z(:,j+1) = acc(1:3,3,i)
        
        Ox = acc(1,4,i);
        Oy = acc(2,4,i);
        Oz = acc(3,4,i);


        origins(:,j+1) = [Ox,Oy,Oz]
    end

    JJ = [];
    J = [];

    final_origin = origins(:,end)

    for k = 2:size(MDH,1)
    
        if jTypes(k) == 0
            Jv(:,k-1) = cross(z(:,k-1),(final_origin-origins(:,k-1)))
        else
            Jv(:,k-1) = z(:,k-1)
        end
        
        if jTypes(k) == 0
            Jw(:,k-1) = z(:,k-1)
        else
            Jw(:,k-1) = [0,0,0]'
        end

        Ji = [Jv(:,k-1),Jw(:,k-1)];

        J = [J Ji];

    end
    JJ(:,:,i) = J
end
disp(JJ)