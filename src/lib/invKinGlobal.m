function Q69 = invKinGlobal(x, y, z,AAA_1_5, Lh,Lf_min,Lg)
    addpath("./lib")

    Pf = [x,y,z,1]' -AAA_1_5*[0,0,0,1]'
    
    if abs(Pf(3))<0.000001
        Pf(3) = 0.000001;
    end

    Q69 = invKin69(Pf(3) ,Pf(1) ,Pf(2) ,Lh,Lf_min,Lg);
    %Q69 = invKin69(0.000001 ,5120 ,0 ,Lh,Lf_min,Lg);

 


end
