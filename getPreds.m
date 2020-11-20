function ctrl = getPreds(pred_horizon,waypoints,v_guess,w_guess,agent_pos,time_sample,theta) %this function performs the optimisation routine using fmincon
    m = size(waypoints);
    options = optimoptions(@fmincon,'Display','iter');
    if (m(1) < pred_horizon) %case to handle when no of waypoints < planning_horizon
        %cost function with regularisation (u -> cols of v,w)
        wpts = waypoints(1:m(1));
        size(wpts)
        %change to distance between waypoint and x(50),y(50)
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-wpts)+0.5*sum(u(:,2).^2);
    else
        wpts = waypoints(1:pred_horizon);
        size(wpts)
        cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon)-wpts)+0.5*sum(u(:,2).^2);
    end
    init = [v_guess w_guess];
%     size(init)
    A = [];
    b = [];
    Aeq = [1;zeros(pred_horizon-1,1);zeros(pred_horizon,1)]';
    Aeq = [Aeq; [zeros(pred_horizon,1);1;zeros(pred_horizon-1,1)]'];
    beq = [v_guess(pred_horizon);w_guess(pred_horizon)];
%     size(Aeq)
%     size(beq)
    %performing optimisation
    v_ulim = 20*ones(pred_horizon,1);
    w_ulim = 0.1*ones(pred_horizon,1);
    v_llim = zeros(pred_horizon,1);
    w_llim = -0.1*ones(pred_horizon,1);
    ulims = [v_ulim w_ulim];
    llims = [v_llim w_llim];
    ctrl = fmincon(cost,init,A,b,Aeq,beq,llims,ulims,[],options);
    
    %amin = -3
    %amax = 2
    %alphamax = 0.1
    %alphamin = -0.1
end