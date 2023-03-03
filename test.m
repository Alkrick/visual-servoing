clear;
close all;

%% params

Pw = [0 0 0]';
Rw = roty(0);
Tw = [Rw Pw];

% Model of a Camera
f = 0.08;
ret_x = 0.1;
ret_y = 0.1;
Pc = [0.1 0.1 0.1]';
Rc = roty(90)*rotz(-90);
Tc = [Rc Pc; 0 0 0 1];

Cammy = PinholeCamera(Pc,Rc,f,ret_x,ret_y);

% Points in World Frame
P1 = [0.7 0.1 0.2]';
P2 = [0.5 0.02 0.05]';
P3 = [0.2 0.5 0.2]';

Points = [P1 P2 P3];
% Get Projection

Proj = Cammy.getProjection(Points);

%% Visualization

Cammy.visualize(Points,Proj);

%% Update test
n = 20;
Pcx = linspace(Pc(1),Pc(1)+0.5,n);
Pcy = linspace(Pc(2),Pc(2),n);
Pcz = linspace(Pc(3),Pc(3),n);
Angx = linspace(0,0,n);
Angy = linspace(0,0,n);
Angz = linspace(0,0,n);

PcUpdate = [Pcx; Pcy; Pcz];
pause()
for i = 1:n
    Pc = PcUpdate(:,i);
    Rc = Cammy.Rc_;
    Cammy=Cammy.update(Pc,Rc);
    Proj=Cammy.getProjection(Points);
    Cammy.visualize(Points,Proj);
end







