clear
close all
clc

addpath("../files");
addpath("./lib")
addpath(getenv("RI_LIB"))

%* -------------------
%* Tree Initialization
%* -------------------
% config.tree_ofset = 4750;
config.tree_ofset = 4750 + 100;

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

%* ---------------------------
%* Computing trajectory points
%* ---------------------------

edge_points = [config.tree_ofset + 1722, 0, 5000]';
first_vertical_point = edge_points;

flip_flop = 1;
TH = 4000;
TB = 3400;
n = 1;

%* Generating edge points
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

% *Generating mid points

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


%* Drawing line
% h_trajectory = line(edge_points(1, :), edge_points(2, :), edge_points(3, :), 'LineWidth', 3, 'Marker', '.', 'MarkerSize', 20);
h_trajectory = line(trajectory_points(1, :), trajectory_points(2, :), trajectory_points(3, :), 'LineWidth', 3, 'Marker', '.', 'MarkerSize', 20);

%* Calculating mid arc points :

mid_arc_points = [];

for i = 1:2:9
    mid_arc_points = [mid_arc_points, [trajectory_points(1, i * 100 + 51), trajectory_points(2, i * 100 + 51), trajectory_points(3, i * 100 + 51)]'];
end

%* Computing phi from coordinates

trajectory_points_w_phi = [];
    
for i = 1:length(trajectory_points)

    x = trajectory_points(1,i);
    y = trajectory_points(2,i);
    z = trajectory_points(3,i);

    Pf = [x; y; z];
    %? Where tf did this came from ?
    vector1 = [-339.99, 3.535, -800]';
    vector2 = Pf - first_vertical_point;

    %!!! Need to round because was giving very small complex numbers when it was close to 1
    phi = acos(round(dot(vector1, vector2) / (norm(vector1) * norm(vector2)), 3));
    cross_vect = cross(vector1, vector2);

    if cross_vect(3) > 0
        phi = -phi;
    end

    trajectory_points_w_phi(1:3,i) = [x,y,z]';
    trajectory_points_w_phi(4:5,i) = 0;
    trajectory_points_w_phi(6,i) = phi;

end



%* ------------------------
%* Loading robot parameters
%* ------------------------
try
    tfid = fopen('tp2.txt');
    tdata = textscan(tfid, ' %s= %s');
    fclose(tfid);

    if (numel(tdata{1}) ~= numel(tdata{2}))
        disp('Error reading file. Missing = !')
        clear tdata tfid
    else
        ndata = {tdata{1} repmat(' = ', size(tdata{1})) tdata{2}};
        sdata = strcat(ndata{1}, ndata{2}, ndata{3});

        for i = 1:numel(sdata)

            try
                eval(sdata{i});
            catch
                sprintf('Bad format in line %d of data file!',i)
            end

        end

        Lf_min = Lf;
        Lf_max = Lf_min *1.7;
        dLf_max = Lf_max - Lf_min;
        d7 = 0;

        clear i tfid ndata tdata sdata
    end

catch
    disp('Cannot open file.')
    disp('Loading default parameters')

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

end

theta_1 = 0;
theta_2 = 0;
theta_3 = 0;
theta_4 = 0;
theta_5 = 0;
theta_6 = 0;
theta_7 = 0;
theta_8 = 0;
theta_9 = 0;

% I would just refactor but matlab doesn't even have a good refactoring :C

dimensions.La = La;
dimensions.Lb = Lb;
dimensions.Lc = Lc;
dimensions.Ld = Ld;
dimensions.Le = Le;
dimensions.Lf_min = Lf_min;
dimensions.Lf_max = Lf_min*1.7;
dimensions.dLf_max = Lf_max - Lf_min;
dimensions.d7 = 0;
dimensions.Lg = 970;
dimensions.Lh = 1150;

%* --------------------
%* Drawing robot joints
%* --------------------

% [F_jointA, V_jointA] = stlread("../models/EloA_Y.STL");
% [F_jointB, V_jointB] = stlread("../models/EloB_-X.STL");
% [F_jointC, V_jointC] = stlread("../models/EloC_-X.STL");
% [F_jointD, V_jointD] = stlread("../models/EloD_-X.STL");
% [F_jointE, V_jointE] = stlread("../models/EloE_-X.STL");

% [F_jointF, V_jointF] = stlread("../models/EloF_min_-Y.STL");
% [F_jointFvar, V_jointFvar] = stlread("../models/EloF_var_-Y.STL");
% [F_jointG, V_jointG] = stlread("../models/EloG_-X.STL");
% % [F_jointH, V_jointH] = stlread("../models/EloH_-X.STL");

% robot.handlers.jointA = patch('Vertices', V_jointA, 'Faces', F_jointA, 'FaceColor', '#1f0aff', 'EdgeAlpha', 0.1);
% robot.handlers.jointB = patch('Vertices', V_jointB, 'Faces', F_jointB, 'FaceColor', '#d91c1c', 'EdgeAlpha', 0.1);
% robot.handlers.jointC = patch('Vertices', V_jointC, 'Faces', F_jointC, 'FaceColor', '#1f0aff', 'EdgeAlpha', 0.1);
% robot.handlers.jointD = patch('Vertices', V_jointD, 'Faces', F_jointD, 'FaceColor', '#d91c1c', 'EdgeAlpha', 0.1);
% robot.handlers.jointE = patch('Vertices', V_jointE, 'Faces', F_jointE, 'FaceColor', '#1f0aff', 'EdgeAlpha', 0.1);
% robot.handlers.jointF = patch('Vertices', V_jointF, 'Faces', F_jointF, 'FaceColor', '#d91c1c', 'EdgeAlpha', 0.1);
% robot.handlers.jointFvar = patch('Vertices', V_jointFvar, 'Faces', F_jointFvar, 'FaceColor', '#d91c1c', 'EdgeAlpha', 0.1);
% robot.handlers.jointG = patch('Vertices', V_jointG, 'Faces', F_jointG, 'FaceColor', '#1f0aff', 'EdgeAlpha', 0.1);
% % robot.handlers.jointH = patch('Vertices', V_jointH, 'Faces', F_jointH, 'FaceColor', '#d91c1c', 'EdgeAlpha', 0.1);

% robot.homogenous_vertices.jointA = [V_jointA'; ones(1, size(V_jointA, 1))];
% robot.homogenous_vertices.jointB = trans(-Lb-200,0,0)*[V_jointB'; ones(1, size(V_jointB, 1))];
% robot.homogenous_vertices.jointC = trans(-Lc-200,0,0)*[V_jointC'; ones(1, size(V_jointC, 1))];
% robot.homogenous_vertices.jointD = trans(-Ld,0,0)*[V_jointD'; ones(1, size(V_jointD, 1))];
% robot.homogenous_vertices.jointE = trans(-Le-200,0,0)*[V_jointE'; ones(1, size(V_jointE, 1))];
% robot.homogenous_vertices.jointF = [V_jointF'; ones(1, size(V_jointF, 1))];
% robot.homogenous_vertices.jointFvar = [V_jointFvar'; ones(1, size(V_jointFvar, 1))];
% robot.homogenous_vertices.jointG =trans(-Lg-200,0,0)*[V_jointG'; ones(1, size(V_jointG, 1))];
% %robot.homogenous_vertices.jointH = [V_jointH'; ones(1, size(V_jointH, 1))];

robot.aa = 0

%* ---------------------------------
%* Declaring DH matrix
%* ---------------------------------

DH = [theta_1, 0, La, -pi / 2;
      -theta_2, Lb, 0, 0;
      theta_3 - pi / 2, Lc, 0, 0;
      theta_4 - pi / 2, Ld, 0, 0;
      theta_5 + pi / 2, Le, 0, 0;
      theta_6, 0, 0, -pi / 2;
      0, 0, Lf_min + d7, pi / 2;
      pi / 2 + theta_8, Lg, 0, pi / 2;
      theta_9, Lh, 0, 0];

%* ---------------------------------
%* Inverse Kinematics Initialization
%* ---------------------------------

jTypes = [0, 0, 0, 0, 0, 0, 1, 0, 0];
sScale = 400;
NN = 100;
% % Initial state of every joint
q1 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';

start_point = 101;
end_point = length(trajectory_points);

% Final state of every joint after first move
Q19 = invKinGlobal(trajectory_points_w_phi(1, start_point), trajectory_points_w_phi(2, start_point), trajectory_points_w_phi(3, start_point),trajectory_points_w_phi(6, start_point), dimensions, first_vertical_point);

theta_1 = Q19(1);

theta_2 = Q19(2);
theta_3 = Q19(3);
theta_4 = Q19(4);
theta_5 = Q19(5);

theta_6 = Q19(6);
d7 = Q19(7);
theta_8 = Q19(8);
theta_9 = Q19(9);

% To get it fixed at the vertical aligned heigth
% theta_2 = -acos(Ld / Lb);
% theta_3 = -theta_2;

q2 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';

QQ = [q1, q2];

q1 = q2;

[H, h, P, AAA] = initRobot(QQ, NN, DH, jTypes, sScale);

waitforbuttonpress
animateRobot(H, AAA, P, h, 0.05, 0, robot);

%* -------------------------------
%* Inverse Kinematics
%* -------------------------------

d7s = [];

for i = start_point + 1:end_point - 1

    Q19 = invKinGlobal(trajectory_points_w_phi(1, i + 1), trajectory_points_w_phi(2, i + 1), trajectory_points_w_phi(3, i + 1),trajectory_points_w_phi(6, i + 1), dimensions, first_vertical_point); 

    theta_1 = Q19(1);

    theta_2 = Q19(2);
    theta_3 = Q19(3);
    theta_4 = Q19(4);
    theta_5 = Q19(5);

    theta_6 = Q19(6);
    d7 = Q19(7);
    theta_8 = Q19(8);
    theta_9 = Q19(9);

    d7s = [d7s d7];

    q2 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8, theta_9]';

    QQ = [q1, q2];

    MDH = generateMultiDH2(DH, QQ, jTypes);
    AAA = calculateRobotMotion(MDH);

    animateRobot(H, AAA, P, h, 0.01, 0, robot)

    q1 = q2;
end

%* -------------------------------
%* Plotting Prismatic Joint Values
%* -------------------------------
plot(d7s)

hold on

d7_limits = 0.7 * dimensions.Lf_min * ones(size(d7s));

plot(d7_limits, '-.')

title('Prismatic Joint Extension')
xlabel('Time')
ylabel('Extension')
