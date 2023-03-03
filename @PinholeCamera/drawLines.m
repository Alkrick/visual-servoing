function  drawLines(obj,Points)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
n = size(Points,2);
for i=1:n
    plot3([Points(1,i) obj.Pc_(1)],[Points(2,i) obj.Pc_(2)],[Points(3,i) obj.Pc_(3)],'k--');
end

end

