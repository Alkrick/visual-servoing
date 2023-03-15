function [v] = computeControlLaw(obj,Le,err,v_)
%COMPUTECONTROLLAW Summary of this function goes here
%   Detailed explanation goes here

Kd = 0.0005*eye(2*obj.n_);
v = pinv(Le)*(-obj.lambda_*err - Kd*Le*v_);

end

