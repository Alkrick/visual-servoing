classdef PinholeCamera
    %PINHOLECAMERA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Pc_
        Eulc_
        Rc_
        Tc_
        
        Pr_
        Rr_
        Tr_
        
        K_
        M_
        
        f_
        ret_x_
        ret_y_
        
        retina_plane_
        fig_
    end
    
    methods
        function obj = PinholeCamera(Pc,eul,f,ret_x,ret_y)
            %PINHOLECAMERA Construct an instance of this class
            %   Detailed explanation goes here
            obj.f_ = f;
            obj.ret_x_ = ret_x;
            obj.ret_y_ = ret_y;
            
            obj.K_ = [f 0 0;
                      0 f 0;
                      0 0 1];
            
            obj=update(obj,Pc,eul);                       
                 
            obj.fig_ = figure();
        end
        %% 
        [imageProj] = getProjection(obj,Points);
        [] = drawLines(obj,Points);
        [] = visualize(obj,varargin);
        function obj=update(obj,Pc,eul)
            obj.Pc_ = Pc;
            obj.Eulc_ = eul;
            obj.Rc_ = eul2rotm(eul');
            obj.Tc_ = [obj.Rc_ Pc; 0 0 0 1];
            
            obj.Pr_ = Pc + obj.Rc_*[0 0 obj.f_]';
            obj.Rr_ = obj.Rc_;
            obj.Tr_ = [obj.Rr_ obj.Pr_; 0 0 0 1];

            obj.M_ = obj.K_ * [eye(3), zeros(3,1)];
            
            Pr_bl = obj.Pr_ + obj.Rr_*[-obj.ret_x_;-obj.ret_y_; 0];
            Pr_tl = obj.Pr_ + obj.Rr_*[-obj.ret_x_; obj.ret_y_; 0];
            Pr_tr = obj.Pr_ + obj.Rr_*[ obj.ret_x_; obj.ret_y_; 0];
            Pr_br = obj.Pr_ + obj.Rr_*[ obj.ret_x_;-obj.ret_y_; 0];
            obj.retina_plane_= [Pr_bl Pr_tl Pr_tr Pr_br];
            
        end
        
        function [] = plotWorldProjection(obj,ProjPoints)
            WProjPoints = obj.Pr_ + obj.Rr_*ProjPoints;
            if size(WProjPoints,1)>0
                plot3(WProjPoints(1,:),WProjPoints(2,:),WProjPoints(3,:),'ko');            
            end
        end
            
    end
end

