function Q = invKinLift(x16, y16, z,La,Lb, Lc, Ld)

    q2 = 2*atan((2*Lb*y16 - (-(La^2 + 2*La*Lc + 2*La*y16 - Lb^2 - 2*Lb*Ld + Lc^2 + 2*Lc*y16 - Ld^2 + x16^2 + y16^2)*(La^2 + 2*La*Lc + 2*La*y16 - Lb^2 + 2*Lb*Ld + Lc^2 + 2*Lc*y16 - Ld^2 + x16^2 + y16^2))^(1/2) + 2*La*Lb + 2*Lb*Lc)/(La^2 + 2*La*Lc + 2*La*y16 + Lb^2 + 2*Lb*x16 + Lc^2 + 2*Lc*y16 - Ld^2 + x16^2 + y16^2)) + 2*pi*0;

    q4 = 2*pi*1 - 2*atan((2*Ld*y16 - (-(La^2 + 2*La*Lc + 2*La*y16 - Lb^2 - 2*Lb*Ld + Lc^2 + 2*Lc*y16 - Ld^2 + x16^2 + y16^2)*(La^2 + 2*La*Lc + 2*La*y16 - Lb^2 + 2*Lb*Ld + Lc^2 + 2*Lc*y16 - Ld^2 + x16^2 + y16^2))^(1/2) + 2*La*Ld + 2*Lc*Ld)/(La^2 + 2*La*Lc + 2*La*y16 - Lb^2 + Lc^2 + 2*Lc*y16 + Ld^2 - 2*Ld*x16 + x16^2 + y16^2));

    q3 = -q2;

    q5 = -q4;

    Q = [q2,q3,q4,q5];
