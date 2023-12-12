function [Q19] = invKinGlobal(x, y, z,phi, dimensions, first_vertical_point)

    addpath("./lib")


    La = dimensions.La;
    Lb = dimensions.Lb;
    Lc = dimensions.Lc;
    Ld = dimensions.Ld;
    Le = dimensions.Le;
    Lf_min = dimensions.Lf_min;
    Lf_max = dimensions.Lf_max;
    dLf_max = dimensions.dLf_max;
    d7 = dimensions.d7;
    Lg = dimensions.Lg;
    Lh = dimensions.Lh;

    joint6_point_abs = [0, 0, 2568.71, 1]';
    P6 = [x, y, z, 1]' - joint6_point_abs;

    if abs(P6(3)) < 0.0001
        P6(3) = 0.0001;
    end

    % Compute tool angle

    Q69 = invKin69(P6(3), P6(1), P6(2), Lh, Lf_min, Lg);
    theta_6 = Q69(1, 1);
    theta_8 = Q69(3, 1);
    Q19(6) = theta_6;

    
    TH = 4000;
    TB = 3400;
    
    heigth = (first_vertical_point(3) - z);
    radius = (heigth / TH * TB) / 2;
    
    Pf = [x; y; z];
    %? Where tf did this came from ?
    vector1 = [-339.99, 3.535, -800]';
    vector2 = Pf - first_vertical_point;

    %!!! Need to round because was giving very small complex numbers when it was close to 1
    % phi = acos(round(dot(vector1, vector2) / (norm(vector1) * norm(vector2)), 3));
    % cross_vect = cross(vector1, vector2);

    % if cross_vect(3) > 0
    %     phi = -phi;
    % end

    % end

    % Compute 2 new angles theta_1 and theta_9
    L_fixed = radius + Lh + 500;
    % Li = (Lf_min + d7) * (cos(abs(theta_6))) +Lg + Lh + radius+500;

    Li = first_vertical_point(1) + 500;
    %L_angled_wrong = (Lf_min + d7) * (cos(abs(theta_6))) + Lg;

    L_angled = sqrt(L_fixed ^ 2 + Li ^ 2 - 2 * L_fixed * Li * cos(phi));

    % Overwritting theta_9 and theta_1 and d7 from q69
    % theta_1 = 0
    theta_1 = asin((sin(phi) * L_fixed) / L_angled);
    theta_9 =- asin((sin(phi) * Li) / L_angled);

    d7 = (L_angled - Lf_min * cos(abs(theta_6)) - Lg) / cos(abs(theta_6));

    Q19(1) = theta_1;
    Q19(7) = d7;
    Q19(8) = theta_8;
    Q19(9) = theta_9;

    % Pd is a RR3d if its vertically aligned

    L1_rr3d = joint6_point_abs(3);
    L2_rr3d = Lf_min + d7;

    Pd = [L2_rr3d * cos(-theta_1) * cos(theta_6) ...
             , L2_rr3d * sin(theta_1) * cos(theta_6) ...
             , L1_rr3d + L2_rr3d * sin(-theta_6)]';

    u = Pd - joint6_point_abs(1:3);
    u = u / norm(u);

    inputMin = 0;
    inputMax = 6000;
    outputMax = 1.3;
    outputMin = 0.5;

    adjustment = (z - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin;
    P_final_lift = joint6_point_abs(1:3) + (Lf_min * (adjustment)) * u;

    %  What to input to the lift

    % Q25 = invKinLift(3000,-2000,0,La,Lb, Lc, Ld)
    Q25 = invKinLift(P_final_lift(1), -P_final_lift(3) + 300, 0, La, Lb, Lc, Ld);

    Q19(7) = d7 - Lf_min * (adjustment);

    theta_2 = Q25(1);
    theta_3 = Q25(2);
    theta_4 = Q25(3);
    theta_5 = Q25(4);

    % theta_2 = 0;
    % theta_3 = 0;
    % theta_4 = 0;
    % theta_5 = 0;

    Q19(2) = theta_2;
    Q19(3) = theta_3;
    Q19(4) = theta_4;
    Q19(5) = theta_5;
end
