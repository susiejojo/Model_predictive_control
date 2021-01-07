function [velo,net] = nonhn_pts(u,agent_pos,agent_goal,theta,time_sample,pred_horizon) %this function 
    x = zeros(pred_horizon,1);
    y = zeros(pred_horizon,1);
%     thetas = zeros(pred_horizon,1);
    x(1) = agent_pos(1);
    y(1) = agent_pos(2);
%     thetas(1) = theta;
    theta_new = theta;
    for i = 2:pred_horizon %returning non-holonomic model predictions with given linear and angular velocities
        theta_new = theta_new + u(i-1,2)*time_sample;
        x(i) = x(i-1) + u(i-1,1)*cos(theta_new)*time_sample;
        y(i) = y(i-1) + u(i-1,1)*sin(theta_new)*time_sample;
%         thetas(i) = theta_new;
    end
%     theta_offset = atan2(agent_goal(2)-x(pred_horizon),agent_goal(1)-y(pred_horizon));
%     velo = [x(pred_horizon);y(pred_horizon);0*theta_new];
    velo = [x(pred_horizon);y(pred_horizon)];
    net = [x,y];
%     size(velo)
%     x,y
end