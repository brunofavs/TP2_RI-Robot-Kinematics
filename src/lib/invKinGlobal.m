function [Q1_69,phi] = invKinGlobal(x, y, z, AAA_1_5, Lh, Lf_min, Lg, mid_arc_points, P0, previous_Q1_69,previous_phi, first_vertical_point)
    addpath("./lib")

    % P6 = [x, y, z, 1]' -AAA_1_5 * [0, 0, 0, 1]';
    P6 = [x, y, z, 1]' - [0, 0, 2568.71, 1]';

    if abs(P6(3)) < 0.0001
        P6(3) = 0.0001;
    end

    % Compute tool angle

    % 1 - Find which arc_point corresponds to the heigth of the trajectory point in question

    arc_point = [];

    for i = 1:length(mid_arc_points)

        if z == mid_arc_points(3, i)
            arc_point = mid_arc_points(:, i);
        end

    end

    Q69 = invKin69(P6(3), P6(1), P6(2), Lh, Lf_min, Lg);
    theta_6 = Q69(1, 1);
    d7 = Q69(2, 1);
    theta_8 = Q69(3, 1);
    theta_9 = Q69(4, 1);

    if isempty(arc_point)

        Q1_69(1) = previous_Q1_69(1);
        Q1_69(2) = theta_6;
        Q1_69(3) = d7;
        Q1_69(4) = theta_8;

        if previous_phi == NaN
            phi = 0
        else
            phi = previous_phi
        end

        try
            Q1_69(5) = previous_Q1_69(5);
        catch
            warning('Previous arc orientation not found');

            Q1_69(5) = 0.53;
            Q1_69(1) = -0.145;
        end

    end

    % Compute tool angle

    P0_adjusted = [P0(1); P0(2); z];
    Pf = [x; y; z];

    if ~isempty(arc_point)
        Pi = arc_point;
    end

    TH = 4000;
    TB = 3400;

    %radius = abs(P0_adjusted(1) - arc_point(1))

    heigth = (first_vertical_point(3) - z)
    radius = (heigth / TH * TB) / 2

    if ~isempty(arc_point)
        vector1 = Pi - P0;
        vector2 = Pf - P0;

        % phi = atan2(norm(cross(vector1,vector2)),dot(vector1,vector2));

        %!!! Need to round because was giving very small complex numbers when it was close to 1
        phi = acos(round(dot(vector1, vector2) / (norm(vector1) * norm(vector2)),3));
        cross_vect = cross(vector1, vector2);

        if cross_vect(3) > 0
            phi = -phi;
        end

    end

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



    % disp("Theta1")
    % disp(rad2deg(theta_1))
    % disp("Theta9")
    % disp(rad2deg(theta_9))

    Q1_69(1) = theta_1;
    Q1_69(2) = theta_6;
    Q1_69(3) = d7;
    Q1_69(4) = theta_8;
    Q1_69(5) = theta_9;

end
