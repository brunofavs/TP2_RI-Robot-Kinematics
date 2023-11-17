function  [H, h, P, AAA] = initRobot(QQ, NN, DH, jTypes, sScale, colors)

if nargin<5
    sScale = 1;
end

if nargin<6
    colors = 'wbgrykwbgryk';
end

[P, F] = seixos3(sScale);

MQ = [];
for k=1:size(QQ, 2)-1
    MQ = [MQ linspaceVect(QQ(:,k), QQ(:,k+1), NN)];
end

MDH = generateMultiDH2(DH, MQ, jTypes);
AAA = calculateRobotMotion(MDH);

h = drawLinks(linkOrigins(AAA(:,:,:,1)));
H = drawFramesColor(AAA(:,:,:,1), P, F, colors);