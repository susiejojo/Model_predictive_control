function ctrl = getPreds(pred_horizon,waypoints,v_guess,w_guess,agent_pos,time_sample,theta,v_last,w_last) %this function performs the optimisation routine using fmincon
    m = size(waypoints);
    options = optimoptions(@fmincon,'Display','iter');
%     if (m(1) < pred_horizon) %case to handle when no of waypoints < planning_horizon
%         %cost function with regularisation (u -> cols of v,w)
% %         wpts = waypoints(1:m(1));
% %         size(wpts)
%         %change to distance between waypoint and x(50),y(50)
% %         cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-wpts)+0.5*sum(u(:,2).^2);
%         waypoints
%         cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-waypoints(:));
%     else
%         wpts = waypoints(1:pred_horizon);
%         size(wpts)
    cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon)-waypoints(:));
%     end
    init = [v_guess w_guess];
%     size(init)
    amin = -3;
    amax = 2;
    alphamax = 0.1;
    alphamin = -0.1;
    A = [diff(eye(pred_horizon)) zeros(pred_horizon-1,pred_horizon)];
    A = [A; [-diff(eye(pred_horizon)) zeros(pred_horizon-1,pred_horizon)]];
    A = [A; [zeros(pred_horizon-1,pred_horizon) diff(eye(pred_horizon))]];
    A = [A; [zeros(pred_horizon-1,pred_horizon) -diff(eye(pred_horizon))]];
    Aeq = [1 zeros(1,pred_horizon*2-1)];
    Aeq = [Aeq; [zeros(1,pred_horizon) 1 zeros(1,pred_horizon-1)]];
    b = [amax*time_sample*ones(pred_horizon-1,1);-amin*time_sample*ones(pred_horizon-1,1);alphamax*time_sample*ones(pred_horizon-1,1);-alphamin*time_sample*ones(pred_horizon-1,1)];
    beq = [v_last;w_last];
%     size(Aeq)
%     size(beq)
    size(b)
    %performing optimisation
    
    v_ulim = 20*ones(pred_horizon,1);
    w_ulim = 0.1*ones(pred_horizon,1);
    v_llim = zeros(pred_horizon,1);
    w_llim = -0.1*ones(pred_horizon,1);
    ulims = [v_ulim w_ulim];
    llims = [v_llim w_llim];
    ctrl = fmincon(cost,init,A,b,Aeq,beq,llims,ulims,[],options);
    
    
end