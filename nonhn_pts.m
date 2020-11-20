function [velo] = nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon) %this function 
    x = zeros(pred_horizon,1);
    y = zeros(pred_horizon,1);
    x(1) = agent_pos(1);
    y(1) = agent_pos(2);
    theta_new = theta;
%     u
    for i = 2:pred_horizon %returning non-holonomic model predictions with given linear and angular velocities
        theta_new = theta_new + u(i-1,2)*time_sample;
        x(i) = x(i-1) + u(i-1,1)*cos(theta_new)*time_sample;
        y(i) = y(i-1) + u(i-1,1)*sin(theta_new)*time_sample;
    end
    velo = [x;y];
%     x,y
end