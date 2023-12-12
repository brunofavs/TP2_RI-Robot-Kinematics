clear all
clc

L1 = 2; L2 = 1.5;
Q = [pi / 2 -pi / 2]';

 DH = [0 L1 0 0
     0 L2 0 0];

jTypes = [0 0]';

NN = 50;
MQ = [];

% for k = 1:size(QQ, 2) - 1
%     MQ = [MQ LinspaceVect(QQ(:, k), QQ(:, k + 1), NN)];
% end

JJ = jacobianGeom(DH,Q,jTypes)
