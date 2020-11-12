function ctrl = getPreds(pred_horizon,waypoints,v_guess,w_guess,agent_pos,time_sample,theta)
%     size(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon))
%     waypoints(1:pred_horizon,:)
    m = size(waypoints);
    options = optimoptions(@fmincon,'Display','iter');
    if (m(1) < pred_horizon)
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-waypoints)+0.5*sum(u(:,2).^2);
    else
%         waypoints(1:pred_horizon,:);
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon)-waypoints(1:pred_horizon,:))+0.5*sum(u(:,2).^2);
    end
    init = [v_guess w_guess];
%     init = v_guess;
    init
%     size(init)
    A = [];
    b = [];
    Aeq = [];
    beq = [];
%     lb = [-0.25];
    ctrl = fmincon(cost,init,A,b,Aeq,beq,[],[],[],options);
end