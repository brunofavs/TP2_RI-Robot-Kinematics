clear
close all
clc

addpath("../files");
addpath("./lib")
addpath(getenv("RI_LIB"))

config.tree_ofset = 5000;


% setViewOptionsDefault();
%setViewOptions([-1000, 10000, -5000, 5000, 0, 3000]);
%* -------------------
%* Robot Initialization
%* -------------------

syms theta_1 ;
syms theta_2 ;
syms theta_3 ;
syms theta_4 ;
syms theta_5 ;
syms theta_6 ;
syms theta_7 ;
syms theta_8 ;
syms theta_9 ;

syms La;
syms Lb;
syms Lc;
syms Ld;
syms Le;
syms Lf_min;
syms Lf_max;
syms dLf_max;
syms d7;
syms Lg;
syms Lh;

DH = [theta_1, 0, La, -pi / 2;
      -theta_2, Lb, 0, 0;
      theta_3 - pi / 2, Lc, 0, 0;
      theta_4 - pi / 2, Ld, 0, 0;
      theta_5 + pi / 2, Le, 0, 0;
      theta_6, 0, 0, -pi / 2;
      0, 0, Lf_min + d7, pi / 2;
      pi / 2 + theta_8, Lg, 0, pi / 2,
      theta_9, Lh, 0, 0];

% R,R
jTypes = [0, 0, 0, 0, 0, 0, 1, 0,0];
sScale = 400;
NN = 100;
return

% Initial state of every joint
q1 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8,theta_9]';

% Final state of every joint

Q = invKinRR3D(Lb*cos(0)*cos(pi/6),0,La+Lb*sin(pi/6),La,Lb)

q2 = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d7, theta_8,theta_9]';

QQ = [q1, q2];

[H, h, P, AAA] = initRobot(QQ, NN, DH, jTypes, sScale);

waitforbuttonpress

animateRobot(H, AAA, P, h, 0.01, 0)
