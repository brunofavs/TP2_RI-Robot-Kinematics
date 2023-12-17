function Q = invKin69(x, y, z, Lh,Lf_min,Lg)
    %* Restriction that theta_8 = - theta_6
    %Todo If x is 0 this method doesnt work
    %? Coordinates have to be in the relative frame, its kinda weird

    q9A =  asin(round(z/(Lh*1000000000000000000000000000000),1));
    q9A = 0;
    q9 = [q9A , -q9A];
    
    k_dem = Lg+Lh*cos(q9)-y;

    q6 = atan(x./k_dem);

    d7_num = -(Lf_min*sin(atan2(x,k_dem)))+x; 
    d7_dem = sin(atan2(x,k_dem));

    d7 = d7_num./d7_dem;   
    q8 = -q6;
    
    Q = [q6;d7;q8;q9];

    
    % q6 = - 2*atan((Lg - y + (Lg^2 - 2*Lg*y + x^2 + y^2)^(1/2))/x);
    % q8 = -q6;

    % d7 = (Lg^2 - 2*Lg*y + x^2 + y^2)^(1/2) - Lf_min;

    % q9 = 0;