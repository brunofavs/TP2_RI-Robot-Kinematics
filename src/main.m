clear
close all
clc

addpath("../files");
addpath("./lib")
addpath(getenv("RI_LIB"))

config.tree_ofset = 4750;

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
%* Points
%* -------------------

edge_points = [config.tree_ofset+1722, 0, 5000]';
first_vertical_point = edge_points;

flip_flop = 1;
TH = 4000;
TB = 3400;
n = 1;

% Generating edge points
for i = 1:10
    radius = (((((TH / 5) * n) / TH) * TB) / 2);
    % x = first_vertical_point(1)-radius;

    x = first_vertical_point(1) - sqrt((radius + 500) ^ 2 - radius ^ 2) + 500;
    y = first_vertical_point(2) + flip_flop * radius;
    z = first_vertical_point(3) - n * (TH / 5);

    if mod(i, 2) == 0
        n = n +1;
    else
        flip_flop = flip_flop * (-1);
    end

    edge_points(:, end + 1) = [x, y, z]';
end
% Generating mid points

linear_generation = 1;

trajectory_points = [first_vertical_point];

n = 1;

for i = 1:10

    radius = (((((TH / 5) * n) / TH) * TB) / 2);

    if linear_generation == 1

        point1 = edge_points(:, i);
        point2 = edge_points(:, i + 1);

        linear_points = linspaceVect(point1, point2, 100);

        trajectory_points = [trajectory_points linear_points];

    else
        % Define two points
        point1 = edge_points(:, i);
        point2 = edge_points(:, i + 1);

        point3 = (point1 + point2) / 2;
        point3(1) = first_vertical_point(1) - radius;

        arc_points = arc3(point1', point3', point2'); % Points for 1 arc
        arc_points = arc_points';

        % Specify the radius of the circle

        trajectory_points = [trajectory_points arc_points];

        if mod(i, 2) == 0
            n = n +1;
        end

    end

    linear_generation = linear_generation * (-1);

end

% h_trajectory = line(edge_points(1, :), edge_points(2, :), edge_points(3, :), 'LineWidth', 3, 'Marker', '.', 'MarkerSize', 20);
h_trajectory = line(trajectory_points(1, :), trajectory_points(2, :), trajectory_points(3, :), 'LineWidth', 3, 'Marker', '.', 'MarkerSize', 20);

% Calculating mid arc points :

mid_arc_points = [];

for i = 1:2:9
    mid_arc_points = [mid_arc_points, [trajectory_points(1, i * 100 + 51), trajectory_points(2, i * 100 + 51), trajectory_points(3, i * 100 + 51)]'];
end

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
% % Initial state of every joint
q1 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';

% theta_2 = -acos(1600/1850);
theta_2 = -acos(Ld/Lb);
theta_3 = -theta_2;

DH_updated = [theta_1, 0, La, -pi / 2;
      -theta_2, Lb, 0, 0;
      theta_3 - pi / 2, Lc, 0, 0;
      theta_4 - pi / 2, Ld, 0, 0;
      theta_5 + pi / 2, Le, 0, 0;
      theta_6, 0, 0, -pi / 2;
      0, 0, Lf_min + d7, pi / 2;
      pi / 2 + theta_8, Lg, 0, pi / 2;
      theta_9, Lh, 0, 0];

AAA_initial = calculateRobotMotion(DH_updated);
AAA_1_5 = AAA_initial(:, :, 1) * AAA_initial(:, :, 2) * AAA_initial(:, :, 3) * AAA_initial(:, :, 4) * AAA_initial(:, :, 5)* AAA_initial(:, :, 6);
Pw = AAA_1_5 * [0, 0, 0, 1]';

Pw = AAA_initial(:,:,1) * [0,0,0,1]'
Pw = AAA_initial(:,:,1) *AAA_initial(:,:,2)  * [0,0,0,1]'

start_point = 101;
end_point = length(trajectory_points);

% Final state of every joint
[Q1_69,previous_phi] = invKinGlobal(trajectory_points(1, start_point), trajectory_points(2, start_point), trajectory_points(3, start_point), AAA_1_5, Lh, Lf_min, Lg, mid_arc_points,first_vertical_point,0,NaN,first_vertical_point)

% theta_4 = Q4; % Only affects z

theta_4 = 0;
theta_5 = -theta_4;

theta_1 = Q1_69(1);
theta_6 = Q1_69(2);
d7      = Q1_69(3);
theta_8 = Q1_69(4);
theta_9 = Q1_69(5);

q2 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';
% q2 = [0, -acos(1600/1850), acos(1600/1850), 0, 0, 0, 0, 0, 0]';

QQ = [q1,q2];

q1 = q2;


[H, h, P, AAA] = initRobot(QQ, NN, DH, jTypes, sScale);
 waitforbuttonpress
animateRobot(H, AAA, P, h, 0.01, 0);

%return

% for i = 1:length(trajectory_points)
%for i = 221:1000
for i = start_point+1:end_point

    % Final state of every joint

    [Q1_69,previous_phi] = invKinGlobal(trajectory_points(1, i+1), trajectory_points(2, i+1), trajectory_points(3, i+1), AAA_1_5, Lh, Lf_min, Lg, mid_arc_points,first_vertical_point,Q1_69,previous_phi,first_vertical_point);


    theta_4 = 0;
    theta_5 = -theta_4;

    theta_1 = Q1_69(1);
    theta_6 = Q1_69(2);
    d7      = Q1_69(3);
    theta_8 = Q1_69(4);
    theta_9 = Q1_69(5);

    q2 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';
    
    
    QQ  = [q1,q2];
    MDH = generateMultiDH2(DH,QQ,jTypes);
    AAA = calculateRobotMotion(MDH);

    animateRobot(H, AAA, P, h, 0.005, 0)

    q1 = q2;
end


