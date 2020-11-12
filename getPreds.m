function ctrl = getPreds(pred_horizon,waypoints,v_guess,w_guess,agent_pos,time_sample,theta) %this function performs the optimisation routine using fmincon
    m = size(waypoints);
    options = optimoptions(@fmincon,'Display','iter');
    if (m(1) < pred_horizon) %case to handle when no of waypoints < planning_horizon
        %cost function with regularisation (u -> cols of v,w)
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-waypoints)+0.1*sum(u(:,2).^2);
    else
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon)-waypoints(1:pred_horizon,:))+0.5*sum(u(:,2).^2);
    end
    init = [v_guess w_guess];
    init
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    %performing optimisation
    ctrl = fmincon(cost,init,A,b,Aeq,beq,[],[],[],options);
end