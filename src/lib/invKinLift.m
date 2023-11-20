function Q = invKinUpLift(x, y, z, Lc, Ld,Le)

    q4 =  asin((Ld + x)/(Lc + Le)) ;

    Q = [q4];
