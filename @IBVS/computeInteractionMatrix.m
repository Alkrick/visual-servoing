function L = computeInteractionMatrix(obj,feature_points,desired_feature_points)
%COMPUTEINTERACTIONMATRIX Summary of this function goes here
%   Detailed explanation goes here
    n = size(feature_points,2);
    L = zeros(2*n,6);
    K = eye(3);%obj.Camera_.K_;
    for i = 1:size(feature_points,2)
        x = feature_points(1,i);
        y = feature_points(2,i);
        Z = feature_points(3,i);

        % Compute the derivative of the image point with respect to the camera pose
        L((2*i-1):2*i,:) = ...
                [K(1,1)/Z, 0, -K(1,1)*x/Z^2, -K(1,1)*x*y/Z^2, K(1,1)+K(1,3)*x/Z^2, -K(1,3)/Z;
                0, K(2,2)/Z, -K(2,2)*y/Z^2, -K(2,2)-K(2,3)*y/Z^2, K(2,3)*x/Z^2, K(2,3)/Z];
        
    end
end

