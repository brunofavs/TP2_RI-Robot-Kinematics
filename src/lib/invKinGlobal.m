function Q1_69 = invKinGlobal(x, y, z,AAA_1_5, Lh,Lf_min,Lg,mid_arc_points,P0)
    addpath("./lib")
    
    
    P6 = [x,y,z,1]' -AAA_1_5*[0,0,0,1]';
    
    if abs(P6(3))<0.000001
        P6(3) = 0.000001;
    end
    % Compute tool angle
    

    % 1 - Find which arc_point corresponds to the heigth of the trajectory point in question
    
    arc_point = [];
    
    for i=1:length(mid_arc_points)
        if z == mid_arc_points(3,i)
            arc_point = mid_arc_points(:,i);
        end
    end
    
    % Compute tool angle
    
    P0_adjusted = [P0(1);P0(2);z];
    Pf = [x;y;z];
    Pi = arc_point;
    
    radius = abs(P0_adjusted(1)-arc_point(1))
    
    vector1 = Pi - P0;
    vector2 = Pf - P0;
    
    phi = atan2(norm(cross(vector1,vector2)),dot(vector1,vector2))
    
    Q69 = invKin69(P6(3) ,P6(1) ,P6(2) ,Lh,Lf_min,Lg)
    
    % Compute 2 new angles theta_1 and theta_9
    
    theta_6 = Q69(1, 1);
    d7 = Q69(2, 1);
    theta_8 = Q69(3, 1);
    
    L_fixed = radius+Lh
    L_angled = (Lf_min+d7)*abs(sin(theta_6))+Lg
    
    Li = L_angled + Lh + radius

    % Overwritting theta_9 from q69
    theta_1 =    asin((sin(phi)*L_fixed)/L_angled);
    theta_9 = - asin((sin(phi)*Li)/L_angled);
    
    disp("Theta1")
    disp(rad2deg(theta_1))
    disp("Theta9")
    disp(rad2deg(theta_9))
    
    Q1_69(1) = theta_1;
    Q1_69(2) = theta_6;
    Q1_69(3) = d7;
    Q1_69(4) = theta_8;
    Q1_69(5) = theta_9;
    
    
    
    
    
    
    
    

end
