classdef IBVS
    %IBVS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        Camera_
        
        iter_ = 200

        n_

        v_ulim_ = 0.05;
        v_llim_ = -0.05;

        tol_ = 2e-3;
        
        lambda_
        
        target_Pw_
        target_Rw_
        target_width_ = 0.05;
        target_length_ = 0.05;
        target_points_
        desired_cPo_
        desired_cRo_
        desired_points_
    end
    
    methods
        function obj = IBVS(Camera,target_center,target_R,desired_center,desired_R)
            %IBVS Construct an instance of this class
            %   Detailed explanation goes here
            obj.Camera_ = Camera;
            
            obj.target_Pw_ = target_center;
            obj.target_Rw_ = target_R;
            obj.desired_cPo_ = desired_center; %% Camera Frame
            obj.desired_cRo_ = desired_R;
            
            target_points = [obj.target_Pw_ + obj.target_Rw_*[-obj.target_width_;-obj.target_length_; 0]
                             obj.target_Pw_ + obj.target_Rw_*[-obj.target_width_; obj.target_length_; 0]
                             obj.target_Pw_ + obj.target_Rw_*[ obj.target_width_; obj.target_length_; 0]
                             obj.target_Pw_ + obj.target_Rw_*[ obj.target_width_;-obj.target_length_; 0]];
            obj.target_points_ =reshape(target_points,[3,4]);
            
            desired_wPo = obj.Camera_.Pc_+obj.Camera_.Rc_*obj.desired_cPo_; % In World Frame
            desired_wRo = obj.Camera_.Rc_*obj.desired_cRo_;
            desired_points = [desired_wPo + desired_wRo*[-obj.target_width_;-obj.target_length_; 0]
                              desired_wPo + desired_wRo*[-obj.target_width_; obj.target_length_; 0]
                              desired_wPo + desired_wRo*[ obj.target_width_; obj.target_length_; 0]
                              desired_wPo + desired_wRo*[ obj.target_width_;-obj.target_length_; 0]];
             obj.desired_points_ =reshape(desired_points,[3,4]);
             
             obj.n_ = size(obj.target_points_,2);
             obj.lambda_ = 0.2 * eye(2*obj.n_);
        end
        
        function [obj] = update(obj)
            desired_wPo = obj.Camera_.Pc_+obj.Camera_.Rc_*obj.desired_cPo_; % In World Frame
            desired_wRo = obj.Camera_.Rc_*obj.desired_cRo_;
            desired_points = [desired_wPo + desired_wRo*[-obj.target_width_;-obj.target_length_; 0]
                              desired_wPo + desired_wRo*[-obj.target_width_; obj.target_length_; 0]
                              desired_wPo + desired_wRo*[ obj.target_width_; obj.target_length_; 0]
                              desired_wPo + desired_wRo*[ obj.target_width_;-obj.target_length_; 0]];
             obj.desired_points_ =reshape(desired_points,[3,4]);
        end
        [obj] = run(obj);
        [L] = computeInteractionMatrix(obj,feature_points,desired_feature_points);
        [v] = computeControlLaw(obj,Le,err,v_);
        function visualizeTargetPoints(obj)
            
             figure(obj.Camera_.fig_);
             subplot(1,2,1);
             s1 = ...
                 fill3(obj.target_points_(1,:),obj.target_points_(2,:),obj.target_points_(3,:),'r','FaceAlpha',0.3);
             s1.EdgeColor = 'none';
             hold on;
             %s2 = ...
                 fill3(obj.desired_points_(1,:),obj.desired_points_(2,:),obj.desired_points_(3,:),'w','FaceAlpha',0.0);
             axis equal;
             obj.Camera_.visualize();
        end
    end
end

