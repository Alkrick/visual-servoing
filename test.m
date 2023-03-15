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
Eulc = [-pi/2 0 -pi/2]';
Tc = [eul2rotm(Eulc') Pc; 0 0 0 1];

Cam = PinholeCamera(Pc,Eulc,f,ret_x,ret_y);

% Points in World Frame
P1 = [0.7 0.1 0.2]';
P2 = [0.5 0.02 0.05]';
P3 = [0.2 0.5 0.2]';

Points = [P1 P2 P3];
% Get Projection

% ProjPoints = Cam.getProjection(Points);

%% Visualization

Cam.visualize(Points);

%% Update test
n = 100;
Pcx = linspace(Pc(1),Pc(1)+0.5,n);
Pcy = linspace(Pc(2),Pc(2),n);
Pcz = linspace(Pc(3),Pc(3),n);
Angx = linspace(0,0,n);
Angy = linspace(0,0,n);
Angz = linspace(pi/18,pi/18,n);

PcUpdate = [Pcx; Pcy; Pcz];
pause()
for i = 1:n
    %Pc = PcUpdate(:,i);
    Eulc = Cam.Eulc_ + [Angz(i) Angy(i) Angx(i)]'; 
    Cam=Cam.update(Pc,Eulc);
    Cam.getProjection(Points);
    Cam.visualize(Points);
end







