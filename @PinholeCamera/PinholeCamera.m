classdef PinholeCamera
    %PINHOLECAMERA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Pc_
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
        
        RetinaPlane_
        fig_
    end
    
    methods
        function obj = PinholeCamera(Pc,Rc,f,ret_x,ret_y)
            %PINHOLECAMERA Construct an instance of this class
            %   Detailed explanation goes here
            obj.f_ = f;
            obj.ret_x_ = ret_x;
            obj.ret_y_ = ret_y;
            
            obj=update(obj,Pc,Rc);
            
            obj.K_ = [f 0 0 0;
                      0 f 0 0;
                      0 0 1 0];
                  
            obj.M_ = diag([f,f,1]) * [eye(3), zeros(3,1)];
            obj.fig_ = figure();
        end
        
        function obj=update(obj,Pc,Rc)
            obj.Pc_ = Pc;
            obj.Rc_ = Rc;
            obj.Tc_ = [Rc Pc; 0 0 0 1];
            
            obj.Pr_ = Pc + Rc*[0 0 obj.f_]';
            obj.Rr_ = Rc;
            obj.Tr_ = [obj.Rr_ obj.Pr_; 0 0 0 1];
            
            Pr_bl = obj.Pr_ + obj.Rr_*[-obj.ret_x_;-obj.ret_y_; 0];
            Pr_tl = obj.Pr_ + obj.Rr_*[-obj.ret_x_; obj.ret_y_; 0];
            Pr_tr = obj.Pr_ + obj.Rr_*[ obj.ret_x_; obj.ret_y_; 0];
            Pr_br = obj.Pr_ + obj.Rr_*[ obj.ret_x_;-obj.ret_y_; 0];
            obj.RetinaPlane_= [Pr_bl Pr_tl Pr_tr Pr_br];
            
        end
        
        ProjPoints = getProjection(obj,Points);
        [] = plotProjection(obj,Points);
        [] = drawLines(obj,Points);
        [] = visualize(obj,Points,Proj);
            
    end
end

