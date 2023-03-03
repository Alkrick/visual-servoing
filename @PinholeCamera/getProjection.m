function [ProjPoints] = getProjection(obj,Points)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
Tc = obj.Tc_;
M = obj.M_;
ProjPoints = [];

for i=1:size(Points,2)
    Ptemp = Tc^-1 * [Points(:,i); 1];
    P_tilde = M*Ptemp;
    ProjPoint = [P_tilde(1)/Ptemp(3); P_tilde(2)/Ptemp(3);0];
    if abs(ProjPoint(1))<obj.ret_x_ && abs(ProjPoint(2))<obj.ret_y_
        ProjPoints(:,i) = ProjPoint
    end
end

