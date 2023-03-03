function plotProjection(obj,ProjPoint)
%PRINTPROJECTION Summary of this function goes here
%   Detailed explanation goes here
n = size(ProjPoint,2);
for i=1:n
    Proj = obj.Pr_ + obj.Rr_*ProjPoint(:,i);
    plot3(Proj(1),Proj(2),Proj(3),'ko');
end

end

