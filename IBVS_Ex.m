clear;
close all;

%% Define Camera 

Pw = [0 0 0]';
Rw = roty(0);
Tw = [Rw Pw];

% Model of a Camera
f = 0.15;
ret_x = 0.1;
ret_y = 0.1;
Pc = [0.1 0.1 0.1]';
Eulc = [-pi/2 0 -pi/2]';
Tc = [eul2rotm(Eulc') Pc; 0 0 0 1];

Cam = PinholeCamera(Pc,Eulc,f,ret_x,ret_y);

%% Define Problem
plane_center = [0.6; 0.3; 0.2]; % In world frame
plane_R = eul2rotm(Eulc');

desired_coord = [0;0;0.2]; % In Camera Frame
desired_R = rotz(0);

Task = IBVS(Cam,plane_center,plane_R,desired_coord,rotz(0));
%% Computation

Task = Task.run();


%% Visualize
Task.visualizeTargetPoints()
