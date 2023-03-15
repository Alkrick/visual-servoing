function visualize(obj,varargin)
%VISUALIZE Summary of this function goes here
%   Detailed explanation goes here


figure(obj.fig_);
subplot(1,2,1);
% s = ...
    fill3(obj.retina_plane_(1,:),obj.retina_plane_(2,:),obj.retina_plane_(3,:),'cyan','FaceAlpha',0.3);
hold on

% s.EdgeColor = 'none';
plotFrame(obj.Pc_,obj.Rc_,"C");
plotFrame([0;0;0],rotz(0),"W");
axis equal

if nargin == 2
    Points = varargin{1};
    n = size(Points,2);
    imageProj = getProjection(obj,Points);
    

    for i = 1:n
        name = sprintf("P%d",i);
        plotFrame(Points(:,i),rotz(0),name);
    end

    obj.drawLines(Points);
    if length(imageProj) > 1
        imageProj(3,:) = 0;
        obj.plotWorldProjection(imageProj);
    end
    axis equal

    subplot(1,2,2)
    
    if length(imageProj) > 1 
        imageProj(3,:) = 0;
        plot(imageProj(1,:),-imageProj(2,:),'bo');
    else
        plot(0,0);
    end
    axis([-0.1 0.1 -0.1 0.1]);
    pause(0.05)
    hold off
    subplot(1,2,1)
    hold off
end
end

