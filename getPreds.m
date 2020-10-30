function ctrl = getPreds(pred_horizon,waypoints,v_guess,w_guess,agent_pos,time_sample,theta)
%     size(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon))
%     waypoints(1:pred_horizon,:)
    m = size(waypoints);
    options = optimoptions(@fmincon,'Display','iter');
    if (m(1) < pred_horizon)
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-waypoints(:,:));
    else
        waypoints(1:pred_horizon,:);
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon)-waypoints(1:pred_horizon,:));
    end
    init = [v_guess,w_guess];
    size(init)
    A = [];
    b = [];
    Aeq = [];
    beq = [];
%     lb = [-0.25];
    ctrl = fmincon(cost,init,A,b,Aeq,beq,[],[],[],options);
end