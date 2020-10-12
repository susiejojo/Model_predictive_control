clc,clear;
% initial guess v and w
planning_horizon = 50;
control_horizon = 10;
v_guess = ones(planning_horizon,1);
w_guess = 0.1*ones(planning_horizon,1);
agent_goal = [100,100];
agent_pos_list = [];
agent_pos = [0,0];
agent_rad = 4;
v_avg = 10;
time_sample = 0.1;
n = v_avg/time_sample;
unif_split = linspace(0,100,n);
waypoints = [];
iter = 1;
theta = atan2(agent_goal(2)-agent_pos(2),agent_goal(1)-agent_pos(1));
% Generating 50 waypoints along the way
for i=1:n
    waypt = [unif_split(i),unif_split(i)];
    waypoints = [waypoints;waypt];
end
for i = 1:control_horizon:n
    ctrl = getPreds(planning_horizon,waypoints(i:n,:),v_guess,w_guess,agent_pos,time_sample,theta);
%     size(ctrl)
    [x,y] = nonhn_pts(ctrl,agent_pos,theta,time_sample,control_horizon);
    agent_pos = [x,y];    
    for j = 1:control_horizon
%         agent_pos(j,:)
        theta = theta + ctrl(j,2)*time_sample;
        F(iter) = plot_figs(agent_pos(j,:),agent_rad,agent_goal,theta);
        agent_pos_list = [agent_pos_list;agent_pos(j,:)];
        basefilename = sprintf('snap%d.png',iter);
        fullname = fullfile('data/',basefilename);
        saveas(F(iter),fullname);
        
        clf;
        hold on;
        plot(waypoints(:,1),waypoints(:,2),"g+");
        plot(agent_pos_list(:,1),agent_pos_list(:,2),'b*');
        iter = iter + 1;
    end
    agent_pos = agent_pos(control_horizon,:);
    agent_pos
    v_guess = [ctrl(11:50,1);ctrl(50,1)*ones(10,1)];
    w_guess = [ctrl(11:50,2);ctrl(50,2)*ones(10,1)];
end

