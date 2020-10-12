function ctrl = getPreds(pred_horizon,waypoints,v_guess,w_guess,agent_pos,time_sample,theta)
%     nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon)
%     waypoints(1:pred_horizon,:)
    cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon)-waypoints(1:pred_horizon,:));
    init = [v_guess,w_guess];
    size(init)
    A = [];
    b = [];
    Aeq = [];
    beq = [];
%     lb = [-0.25];
    ctrl = fmincon(cost,init,A,b,Aeq,beq);
%     ctrl
end