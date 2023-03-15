CAMERA MODEL
function projection = proj(f, O, pose, p)

d = pose(1:3) - O;    % camera's position in world frame
ang = pose(4:6);      % camera's orientation 
p1 = p(1:3);
p2 = p(4:6);

%%
Rx = [1   0       0   ;
      0 cos(ang(1)) -sin(ang(1));
      0 sin(ang(1)) cos(ang(1))];

Ry = [cos(ang(2)) 0 sin(ang(2)) ;
       0     1   0    ;
     -sin(ang(2)) 0 cos(ang(2))];
 
Rz = [cos(ang(3)) -sin(ang(3)) 0;
      sin(ang(3))  cos(ang(3)) 0;
       0      0     1];


Rc = Rz * Ry * Rx;


Twc = [Rc            d;
      zeros(1,3)     1];

%%
P = diag([f,f,1]) * [eye(3), zeros(3,1)]; % camera projection matrix

p_cam1 = Twc^-1 * [p1; 1];   % point's position in camera frame
p_cam2 = Twc^-1 * [p2; 1];

p_tilde1 = P * p_cam1;
p_tilde2 = P * p_cam2;

x_proj1 = p_tilde1(1)/p_cam1(3);
y_proj1 = p_tilde1(2)/p_cam1(3);
Zc1 = p_cam1(3);

x_proj2 = p_tilde2(1)/p_cam2(3);
y_proj2 = p_tilde2(2)/p_cam2(3);
Zc2 = p_cam2(3);


projection  = [x_proj1; y_proj1; Zc1; x_proj2; y_proj2; Zc2]; %; x_proj2; y_proj2; Zc2];

end

%CONTROL

function u = fcn(err, projection, pose, Kp, p2, Kd, q_dot, p1)

% projection = [x1 y1 Zc1; x2 y2 Zc2]
x1 = projection(1);         % x projection in the image plane
y1 = projection(2);         % y projection
Zc1 = projection(3);
x2 = projection(4);
y2 = projection(5);
Zc2 = projection(6);
ang = pose(4:6);  % camera's orientation 

if Zc1 == 0 || isnan(Zc1)
    Zc1 = p1(3)- pose(3);
end

if Zc2 == 0 || isnan(Zc2)
    Zc2 = p2(3) - pose(3);
end

% Interaction matrix 
Ls1 = [ -1/Zc1     0     x1/Zc1     x1*y1    -(1+x1^2)     y1;  
        0       -1/Zc1   y1/Zc1    1+y1^2      -x1*y1     -x1];
    
Ls2 = [ -1/Zc2     0     x2/Zc2     x2*y2    -(1+x2^2)     y2;  
        0       -1/Zc2   y2/Zc2    1+y2^2      -x2*y2     -x2];
    
Ls = [Ls1; Ls2];

% Trasformation
Rx = [1   0       0   ;
      0 cos(ang(1)) -sin(ang(1));
      0 sin(ang(1)) cos(ang(1))];

Ry = [cos(ang(2)) 0 sin(ang(2)) ;
       0     1   0    ;
     -sin(ang(2)) 0 cos(ang(2))];
 
Rz = [cos(ang(3)) -sin(ang(3)) 0;
      sin(ang(3))  cos(ang(3)) 0;
       0      0     1];

Rc = Rz * Ry * Rx;

J2 = [1     sin(ang(1))*tan(ang(2))    cos(ang(1))*tan(ang(2));
      0           cos(ang(1))                  -sin(ang(1));
      0     sin(ang(1))/cos(ang(2))    cos(ang(1))/cos(ang(2)) ];

J = eye(6);  % Jacobian J(q)
  
Jl = Ls * [Rc' zeros(3); zeros(3) J2^-1] * J;

% IBVS control Law
u = pinv(Jl) * (Kp * err - Kd * Jl * q_dot);

%MANIPULATOR

function pose = fcn(pose, u, dT)

position = zeros(3, 1);
angle = zeros(3, 1);
ang = u(4:6);

Rx = [1   0       0   ;
      0 cos(ang(1)) -sin(ang(1));
      0 sin(ang(1)) cos(ang(1))];

Ry = [cos(ang(2)) 0 sin(ang(2)) ;
       0     1   0    ;
     -sin(ang(2)) 0 cos(ang(2))];
 
Rz = [cos(ang(3)) -sin(ang(3)) 0;
      sin(ang(3))  cos(ang(3)) 0;
       0      0     1];

J1 = Rz * Ry * Rx;

J2 = [1     sin(ang(1))*tan(ang(2))    cos(ang(1))*tan(ang(2));
      0           cos(ang(1))                  -sin(ang(1));
      0     sin(ang(1))/cos(ang(2))    cos(ang(1))/cos(ang(2)) ];


position = pose(1:3) + J1 * u(1:3) * dT;
angle = pose(4:6) + J2 * ang * dT;
pose = [position; angle];


%%%
function J = compute_image_jacobian(K, T, X)
% Compute the image Jacobian using the exact Jacobian calculation method
% Inputs:
%   - K: 3x3 camera intrinsic matrix
%   - T: 4x4 homogeneous transformation matrix of the end-effector
%   - X: 3xn matrix representing the 3D coordinates of n feature points in the world frame
% Output:
%   - J: 2nx6 image Jacobian matrix

% Extract rotation matrix and translation vector from the transformation matrix
R = T(1:3, 1:3);
t = T(1:3, 4);

% Compute the world-to-camera transformation matrix
T_wc = [R' -R'*t; 0 0 0 1];

% Compute the camera-to-image projection matrix
P_ci = K * T_wc;

% Compute the image Jacobian
J = zeros(2*size(X,2), 6);
for i = 1:size(X,2)
    % Compute the homogeneous coordinates of the feature point in the world frame
    x_w = [X(:,i); 1];
    
    % Compute the homogeneous coordinates of the feature point in the camera frame
    x_c = T_wc * x_w;
    
    % Compute the derivative of the image point with respect to the camera pose
    dxdT = [K(1,1)/x_c(3), 0, -K(1,1)*x_c(1)/x_c(3)^2, -K(1,1)*x_c(1)*x_c(2)/x_c(3)^2, K(1,1)+K(1,3)*x_c(1)/x_c(3)^2, -K(1,3)/x_c(3);
            0, K(2,2)/x_c(3), -K(2,2)*x_c(2)/x_c(3)^2, -K(2,2)-K(2,3)*x_c(2)/x_c(3)^2, K(2,3)*x_c(1)/x_c(3)^2, K(2,3)/x_c(3)];
    
    % Compute the derivative of the camera pose with respect to the end-effector pose
    dTdE = [R', -R'*skew3(t);
            zeros(3), R'];
    
    % Compute the derivative of the feature point with respect to the end-effector pose
    dXdE = dTdE * x_w;
    
    % Compute the image Jacobian for this feature point
    J((2*i-1):2*i,:) = dxdT * P_ci * dXdE';
end


