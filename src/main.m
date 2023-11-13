clear
close all
clc

addpath("../files");
addpath("./lib")
% addpath(getenv("RI_LIB"))

% Import the VRML file (or the appropriate file format)
[F_log,V_log] = stlread("../models/Whole_Tree - Log-1.STL");
[F_cone,V_cone] = stlread("../models/Whole_Tree - Tree_Cone-1.STL");
[F_aux_cone,V_aux_cone] = stlread("../models/Whole_Tree - Auxiliar_Cone-1.STL");
% Create a figure
figure;


% Plot the 3D model using patch
tree.handlers.log = patch('Vertices', V_log, 'Faces', F_log,'FaceColor',' #575b2c','EdgeAlpha',0);
tree.handlers.cone = patch('Vertices', V_cone, 'Faces', F_cone,'FaceColor','#11ff14 ','EdgeAlpha',0);
tree.handlers.aux_cone = patch('Vertices', V_aux_cone, 'Faces', F_aux_cone,'FaceColor',' #34f637','FaceAlpha',0.1,'EdgeAlpha',0);

axis equal;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;


% Show the plot
title('Your 3D Model');
