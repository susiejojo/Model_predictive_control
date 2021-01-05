clc,clearvars -except v_list1 w_list1;

% setting planning and control horizon
planning_horizon = 50;
control_horizon = 10;

% setting initial parameters
v_guess = ones(planning_horizon,1);
w_guess =  -0.06 + (0.06+0.06)*rand(planning_horizon,1);

%setting goal at [125,125] and initial position
agent_pos = [0,0];
agent_goal = [50,50];
end_orientation = 0;
obst_x = 20;
obst_y = 20;
obst_pos = [obst_x,obst_y];
obst_theta = 0;
obst_v = 5*ones(planning_horizon,1);
obst_w = ones(planning_horizon,1);

agent_pos_list = []; %stores the agent_positions wrt time for ease of plotting
v_list = []; %stores the linear velocities wrt time for ease of plotting
w_list = []; %stores the angular velocities wrt time for ease of plotting
theta_list = []; %stores the headings wrt time for ease of plotting
waypoints = []; %list of waypoints of length n
cost_list = []; %list of cost function values

%setting other parameters like radius, average velocity, time sample
agent_rad = 5;
obst_rad = 5;
% v_avg = 5;
time_sample = 0.1;
% n = 81; %no of waypoints = (n-1) = 125

chckpt = agent_pos; %this indicates the start of each control horizon
theta_chk = atan2(agent_goal(2)-agent_pos(2),agent_goal(1)-agent_pos(1)); %initial value of heading along the intended direction 
% theta_chk = 0;
% theta_chk

% generating waypoints
% unif_split = 0:125;
iter = 1; %used to keep track of frames being stored as images to help in generating video

waypoints = agent_goal;
% Generating 50 waypoints along the way
% for i=1:n
%     waypt = [unif_split(i),unif_split(i)];
%     waypoints = [waypoints;waypt];
% end

has_obstacle = 1;
has_lane_con = 0;
v_last = 1;
w_last = w_guess(control_horizon);

lane_var = 0:0.01:agent_goal(1);
left_lane = lane_var + 25;
right_lane = lane_var - 25;

while (norm(agent_pos - agent_goal)>0.5) %main loop, plan every 1,11,21... time instant and execute motion for time_instants = control_horizon
    %get predictions for time_steps = planning_horizon,
    % in case no of waypoints < planning horizon, 
    % get predictions for remaining waypoints
    if (has_obstacle)
        obst_theta_dummy = obst_theta;
        for i = 1:planning_horizon
            obst_w(i) = (pi/4-obst_theta_dummy)/time_sample;
            obst_theta_dummy = obst_theta_dummy + obst_w(i)*time_sample;
        end
    end
    [ctrl,cost] = getPreds(planning_horizon,waypoints,end_orientation,v_guess,w_guess,chckpt,agent_goal,time_sample,theta_chk,v_last,w_last,has_obstacle,has_lane_con,obst_pos,obst_v,obst_w,obst_rad,obst_theta,agent_rad);
    % setting theta to the theta obtained at the end of the last control
    % horizon
    theta = theta_chk;
    agent_pos_init = agent_pos;
    cost_list = [cost_list,cost];
    %going over time_steps = control_horizon
    for j = 1:control_horizon
        if (norm(agent_pos - agent_goal)<0.5)
            break
        else
%             norm(agent_pos- agent_goal)
        %ctrl(:,1) is list of 50 linear velocities, 
        %ctrl(:,2) is list of 50 angular velocities
        %we take only the first 10 from each list to execute motion
            theta = theta + ctrl(j,2)*time_sample; %updating heading
%             theta
            agent_pos(1) = agent_pos(1) + ctrl(j,1)*cos(theta)*time_sample; % x coordinate
            agent_pos(2) = agent_pos(2) + ctrl(j,1)*sin(theta)*time_sample; % y coordinate
%             agent_pos
            waypts_lim = 100; %sets the axes dimensions for plotting
            % plotting the simulation
            F(iter) = plot_figs(agent_pos,agent_rad,agent_goal,theta,waypts_lim,obst_rad,obst_pos,obst_theta,has_obstacle);
%             obst_w_now = (pi-obst_theta)/time_sample;
            obst_w
            obst_theta = obst_theta + obst_w(j)*time_sample;
            obst_x = obst_x + obst_v(j)*cos(obst_theta)*time_sample;
            obst_y = obst_y + obst_v(j)*sin(obst_theta)*time_sample;
            obst_pos = [obst_x,obst_y];
            obst_pos
            plot(chckpt(1),chckpt(2),'r*','markersize',25);
            %appending to the list of linear and angular velocities, and
            %headings
            v_list = [v_list;ctrl(j,1)];
            w_list = [w_list;ctrl(j,2)];
            theta_list = [theta_list;theta];
            agent_pos_list = [agent_pos_list;agent_pos];
            basefilename = sprintf('snap%d.png',iter);
            fullname = fullfile('data/',basefilename);
            saveas(F(iter),fullname);
            clf;
            hold on;
            [~,planner] = nonhn_pts(ctrl,agent_pos_init,agent_goal,theta_chk,time_sample,planning_horizon);
            plot(planner(:,1),planner(:,2),"go");
            plot(waypoints(:,1),waypoints(:,2),"g+");
            plot(agent_pos_list(:,1),agent_pos_list(:,2),'b*');
            if (has_lane_con)
                plot(lane_var,left_lane,"k--");
                plot(lane_var,right_lane,"k--");
            end
            iter = iter + 1;
        end
        
    end
%     agent_pos
    %updating theta_chk as theta_checkpoint at the end of each control
    %horizon
    theta_chk = theta;
    %updating checkpoint position for plotting the red X
    chckpt = agent_pos;
    v_last = ctrl(control_horizon,1);
    w_last = ctrl(control_horizon,2);
    
    %updating the v_guess and w_guess with 11:50 from predictions, and the
    %last 10 as prediction(50)
    v_guess = [ctrl(control_horizon+1:planning_horizon,1);ctrl(planning_horizon,1)*ones(control_horizon,1)];
    w_guess = [ctrl(control_horizon+1:planning_horizon,2);ctrl(planning_horizon,2)*ones(control_horizon,1)];
end
figure;
plot(v_list,'r-');
hold on;
plot(w_list,'m-');
figure;
plot(cost_list,'r-');
v_list2 = v_list;
w_list2 = w_list;
title("Cost function plot");
