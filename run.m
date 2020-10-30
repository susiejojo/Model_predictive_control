clc,clear;
% initial guess v and w
planning_horizon = 50;
control_horizon = 10;
v_guess = ones(planning_horizon,1);
w_guess = 0.1*ones(planning_horizon,1);
agent_goal = [10,10];
agent_pos_list = [];
v_list = [];
w_list = [];
agent_pos = [0,0];
agent_rad = 1;
v_avg = 5;
time_sample = 0.1;
n = v_avg/time_sample;
unif_split = linspace(0,10,n);
waypoints = [];
theta_list = [];
iter = 1;
theta = atan2(agent_goal(2)-agent_pos(2),agent_goal(1)-agent_pos(1));
% Generating 50 waypoints along the way
for i=1:n
    waypt = [unif_split(i),unif_split(i)];
    waypoints = [waypoints;waypt];
end
for i = 1:control_horizon:n
%     i
    ctrl = getPreds(planning_horizon,waypoints(i:n,:),v_guess,w_guess,agent_pos,time_sample,theta);
%     size(ctrl)
%     [x,y] = nonhn_pts(ctrl,agent_pos,theta,time_sample,control_horizon);
%     agent_pos = [x,y];    
    for j = 1:control_horizon
%         agent_pos(j,:)
        theta = theta + ctrl(j,2)*time_sample;
        agent_pos(1) = agent_pos(1) + ctrl(j,1)*cos(theta)*time_sample;
        agent_pos(2) = agent_pos(2) + ctrl(j,1)*sin(theta)*time_sample;
        agent_pos
        waypts_lim = 10;
        F(iter) = plot_figs(agent_pos,agent_rad,agent_goal,theta,waypts_lim);
        v_list = [v_list;ctrl(j,1)];
        w_list = [w_list;ctrl(j,2)];
        theta_list = [theta_list;theta];
        agent_pos_list = [agent_pos_list;agent_pos];
        basefilename = sprintf('snap%d.png',iter);
        fullname = fullfile('data/',basefilename);
        saveas(F(iter),fullname);
        clf;
        hold on;
        plot(waypoints(:,1),waypoints(:,2),"g+");
        plot(agent_pos_list(:,1),agent_pos_list(:,2),'b*');
        iter = iter + 1;
    end
%     agent_pos = agent_pos(control_horizon,:);
    agent_pos
    v_guess = [ctrl(control_horizon+1:planning_horizon,1);ctrl(planning_horizon,1)*ones(control_horizon,1)];
    w_guess = [ctrl(control_horizon+1:planning_horizon,2);ctrl(planning_horizon,2)*ones(control_horizon,1)];
end
figure;
plot(v_list,'r-');
hold on;
plot(w_list,'m-');
