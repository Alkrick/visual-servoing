function obj = run(obj)
%RUN Summary of this function goes here
%   Detailed explanation goes here

    %% Transformation Matrices
%     wTc = [obj.Camera_.Rc_ obj.Camera_.Pc_; 0 0 0 1];
%     wTt = [obj.target_Rw_ obj.target_Pw_; 0 0 0 1];
%     
%     cTo = wTc^-1 * wTt;
%     cdTo = [obj.desired_cRo_ obj.desired_cPo_; 0 0 0 1];
    
    %% Feature Builder
     
%     pd = obj.desired_points_;
%     obj.Camera_.visualize(pd);
    
%     p = obj.target_points_;
%     obj.Camera_.visualize(p);
    
    
    %% Control Loop
    err =1;
    camera_velocity = zeros(6,1);
    pause
    while norm(err) > obj.tol_
        tc = obj.Camera_.Pc_;
        Rc = obj.Camera_.Rc_;
        
        pd = obj.Camera_.getProjection(obj.desired_points_);
%         obj.Camera_.visualize(obj.desired_points_);
        
        p = obj.Camera_.getProjection(obj.target_points_);
%         obj.Camera_.visualize([obj.desired_points_ obj.target_points_]);
        
        
        %%% Interaction Matrix and Jacobian
 
        Lx = obj.computeInteractionMatrix(p,pd);
        

        pd = pd(1:2,:);
        p = p(1:2,:);
        pd = reshape(pd,[],1);
        p = reshape(p,[],1);


        %%% Compute Error
        err = pd-p;
        norm(err)
     
        %%% Compute Control Law 
        camera_velocity = obj.computeControlLaw(Lx,err,camera_velocity);
        camera_velocity = max(obj.v_llim_,min(obj.v_ulim_,camera_velocity));
        %%% Update Cam  era
        Pc = obj.Camera_.Pc_ + obj.Camera_.Rc_*camera_velocity(1:3);
        Eulc = obj.Camera_.Eulc_ + obj.Camera_.Rc_*camera_velocity(4:6);
        obj.Camera_ = obj.Camera_.update(Pc,Eulc);
        obj = obj.update();
        obj.Camera_.visualize([obj.desired_points_ obj.target_points_]);
        pause(0.001)
        
    end
    
end

