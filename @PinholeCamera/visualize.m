function visualize(obj,Points,Proj)
%VISUALIZE Summary of this function goes here
%   Detailed explanation goes here

n = size(Points,2);
figure(obj.fig_);
subplot(1,2,1);
s = fill3(obj.RetinaPlane_(1,:),obj.RetinaPlane_(2,:),obj.RetinaPlane_(3,:),'r','FaceAlpha',0.3);
hold on

% s.EdgeColor = 'none';
plotFrame(obj.Pc_,obj.Rc_,"C");
plotFrame([0;0;0],rotz(0),"W");

for i = 1:n
    name = sprintf("P%d",i);
    plotFrame(Points(:,i),rotz(0),name);
end

obj.drawLines(Points);
obj.plotProjection(Proj);
axis equal

subplot(1,2,2)
ProjWorld = Proj;
plot(ProjWorld(1,:),-ProjWorld(2,:),'bo');
axis([-0.1 0.1 -0.1 0.1]);
pause()
hold off
subplot(1,2,1)
hold off

end

