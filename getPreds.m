function [ctrl,cost_calc] = getPreds(pred_horizon,waypoints,end_orientation,v_guess,w_guess,agent_pos,agent_goal,time_sample,theta,v_last,w_last,has_obstacle,has_lane_con,obst_pos,obst_rad,agent_rad) %this function performs the optimisation routine using fmincon
    m = size(waypoints);
    options = optimoptions(@fmincon,'MaxFunctionEvaluations',30000,'MaxIterations',10000);
%     if (m(1) < pred_horizon) %case to handle when no of waypoints < planning_horizon
%         %cost function with regularisation (u -> cols of v,w)
% %         wpts = waypoints(1:m(1));
%         %change to distance between waypoint and x(50),y(50)
% %         cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-wpts)+0.5*sum(u(:,2).^2);
%         waypoints
%         cost = @(u)norm(nonhn_pts(u,agent_pos,theta,time_sample,m(1))-waypoints(:));
%     else
%         wpts = waypoints(1:pred_horizon);
    cost = @(u)norm(nonhn_pts(u,agent_pos,agent_goal,theta,time_sample,pred_horizon)-[waypoints(:);20*end_orientation]);
%     end
    init = [v_guess w_guess];
    amin = -2;
    amax = 3;
    alphamax = 0.2;
    alphamin = -0.2;
    A = [diff(eye(pred_horizon)) zeros(pred_horizon-1,pred_horizon)];
    A = [A; [-diff(eye(pred_horizon)) zeros(pred_horizon-1,pred_horizon)]];
    A = [A; [zeros(pred_horizon-1,pred_horizon) diff(eye(pred_horizon))]];
    A = [A; [zeros(pred_horizon-1,pred_horizon) -diff(eye(pred_horizon))]];
    Aeq = [1 zeros(1,pred_horizon*2-1)];
    Aeq = [Aeq; [zeros(1,pred_horizon) 1 zeros(1,pred_horizon-1)]];
    b = [amax*time_sample*ones(pred_horizon-1,1);-amin*time_sample*ones(pred_horizon-1,1);alphamax*time_sample*ones(pred_horizon-1,1);-alphamin*time_sample*ones(pred_horizon-1,1)];
    beq = [v_last;w_last];
    radii_sum = obst_rad + agent_rad;
    collisionconst = @(x)colnfn(x,obst_pos,has_lane_con,has_obstacle,radii_sum,agent_goal,agent_pos,theta,time_sample,pred_horizon);
    %performing optimisation
    
    v_ulim = 20*ones(pred_horizon,1);
    w_ulim = 0.5*ones(pred_horizon,1);
    v_llim = zeros(pred_horizon,1);
    w_llim = -0.5*ones(pred_horizon,1);
    ulims = [v_ulim w_ulim];
    llims = [v_llim w_llim];
    if (has_obstacle==0 && has_lane_con==0)
        [ctrl,fval,exitflag,output] = fmincon(cost,init,A,b,Aeq,beq,llims,ulims,[],options);
    else
        [ctrl,fval,exitflag,output] = fmincon(cost,init,A,b,Aeq,beq,llims,ulims,collisionconst,options);
    end
    fval, exitflag, output
    cost_calc = feval(cost,ctrl);
    
end