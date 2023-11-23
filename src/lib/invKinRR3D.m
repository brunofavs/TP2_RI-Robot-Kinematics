function Q = invKinRR3D(x, y, z, L1, L2)

    q1 = atan2(y, x);

    q2 = -asin((z - L1) / L2);

    if imag(q2) ~= 0
        disp("Warning")
        q2 = -sign(z - L1) * pi / 2
    end


    Q = [q1; q2];
