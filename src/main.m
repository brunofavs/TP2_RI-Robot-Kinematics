clear
close all
clc

addpath("../files");
addpath("./lib")
addpath(getenv("RI_LIB"))

config.tree_ofset = 5000;

%* -------------------
%* Tree Initialization
%* -------------------

% Import the VRML file (or the appropriate file format)
[F_log, V_log] = stlread("../models/Whole_Tree - Log-1.STL");
[F_cone, V_cone] = stlread("../models/Whole_Tree - Tree_Cone-1.STL");
[F_aux_cone, V_aux_cone] = stlread("../models/Whole_Tree - Auxiliar_Cone-1.STL");
% Create a figure
figure;

% Plot the 3D model using patch
tree.handlers.log = patch('Vertices', V_log, 'Faces', F_log, 'FaceColor', ' #575b2c', 'EdgeAlpha', 0);
tree.handlers.cone = patch('Vertices', V_cone, 'Faces', F_cone, 'FaceColor', '#11ff14 ', 'EdgeAlpha', 0.1);
tree.handlers.aux_cone = patch('Vertices', V_aux_cone, 'Faces', F_aux_cone, 'FaceColor', ' #34f637', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);

V_log_homogenous = [V_log'; ones(1, size(V_log, 1))];
V_cone_homogenous = [V_cone'; ones(1, size(V_cone, 1))];
V_aux_cone_homogenous = [V_aux_cone'; ones(1, size(V_aux_cone, 1))];

initial_tree_transform = trans(config.tree_ofset, 0, 0) * trans(0, 2222, 0) * rotx(pi / 2);
V_log_homogenous = initial_tree_transform * V_log_homogenous;
V_cone_homogenous = initial_tree_transform * V_cone_homogenous;
V_aux_cone_homogenous = initial_tree_transform * V_aux_cone_homogenous;

tree.handlers.log.Vertices = V_log_homogenous(1:3, :)';
tree.handlers.cone.Vertices = V_cone_homogenous(1:3, :)';
tree.handlers.aux_cone.Vertices = V_aux_cone_homogenous(1:3, :)';

% setViewOptionsDefault();
setViewOptions([-1000, 10000, -5000, 5000, 0, 3000]);
%* -------------------
%* Robot Initialization
%* -------------------

theta_1 = 0;
theta_2 = 0;
theta_3 = 0;
theta_4 = 0;
theta_5 = 0;
theta_6 = 0;
theta_7 = 0;
theta_8 = 0;
theta_9 = 0;

La = 940;
Lb = 1850;
Lc = 400;
Ld = 1600;
Le = 300;
Lf_min = 1800;
Lf_max = 3000;
dLf_max = Lf_max - Lf_min;
d7 = 0;
Lg = 970;
Lh = 1150;

DH = [theta_1, 0, La, -pi / 2;
      -theta_2, Lb, 0, 0;
      theta_3 - pi / 2, Lc, 0, 0;
      theta_4 - pi / 2, Ld, 0, 0;
      theta_5 + pi / 2, Le, 0, 0;
      theta_6, 0, 0, -pi / 2;
      0, 0, Lf_min + d7, pi / 2;
      pi / 2 + theta_8, Lg, 0, pi / 2;
      theta_9, Lh, 0, 0];

% R,R
jTypes = [0, 0, 0, 0, 0, 0, 1, 0, 0];
sScale = 400;
NN = 100;

AAA_initial = calculateRobotMotion(DH);
AAA_1_5 = AAA_initial(:,:,1)*AAA_initial(:,:,2)*AAA_initial(:,:,3)*AAA_initial(:,:,4)*AAA_initial(:,:,5)
Pw = AAA_1_5 * [0,0,0,1]';

% Initial state of every joint
q1 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';

% Final state of every joint

Q12 = invKinRR3D(Lb * cos(0) * cos(pi / 8), 0, La + Lb * sin(pi / 8), La, Lb)

Q4 = invKinLift(-Ld * cos(pi / 4), -Ld * sin(pi / 4) + Le, 0, Lc, Ld, Le);

%Q69 = invKin69  (863,3699,0,   Lh,Lf_min,Lg)
%Q69 = invKinGlobal(5370,0,1640,AAA_1_5,Lh,Lf_min,Lg)
Q69 = invKinGlobal(5300,520,1040,AAA_1_5,Lh,Lf_min,Lg)
% theta_1 = Q12(1);
% theta_2 = Q12(2);
theta_1 = 0;
%theta_2 = 0;
theta_3 = -theta_2;
% theta_4 = Q4; % Only affects z
theta_4 = 0;
theta_5 = -theta_4;

% theta_6 = -0.5;
% % d7 = dLf_max;
% d7 = 0;
% theta_8 = -theta_6;
% theta_9 = 0;

theta_6 = Q69(1,1);
d7      = Q69(2,1);
theta_8 = Q69(3,1);
theta_9 = Q69(4,1);



q2 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';

QQ = [q1, q2];





[H, h, P, AAA] = initRobot(QQ, NN, DH, jTypes, sScale);

waitforbuttonpress

animateRobot(H, AAA, P, h, 0.01, 0)
