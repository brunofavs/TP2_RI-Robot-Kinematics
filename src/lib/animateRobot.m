function animateRobot(H, AAA, P, h, sd, plotpath)

for k = 1:size(AAA,4) %For every frame
    org = linkOrigins(AAA(:,:,:,k)); %Get the links origins

    %Update the line linking the links
    
    h.XData = org(1,:);
    h.YData = org(2,:);
    h.ZData = org(3,:);

    %Calculate the transformation for each link based on the previous links
    T = eye(4);
    for j=1:size(AAA,3)
        T = T * AAA(:,:,j,k);
        Pk = T*P;
        set(H(j),'Vertices', Pk(1:3,:)'); %Update the H(j) patch
    end
    
    if plotpath
        X=T(1,4); Y=T(2,4); Z=T(3,4);
        plot3(X,Y,Z, '.r', 'MarkerSize',6);
    end
    
   
    pause(sd)
end

end