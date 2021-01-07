function f = plot_figs(agent_pos,agent_rad,agent_goal,theta,waypts_lim,obst_rad,obst_pos,obst_theta,has_obstacle)
    axis([-10 waypts_lim+10 -10 waypts_lim+10]);
    R= ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
    L = agent_rad;
    H = agent_rad;
    X=([-L/2, L/2, L/2, -L/2]);
    Y=([-H/2, -H/2, H/2, H/2]);
    for i=1:4
        T(:,i)=R*[X(i); Y(i)];
    end
    center1 = agent_pos(1);
    center2 = agent_pos(2);
    x_lower_left=center1+T(1,1);
    x_lower_right=center1+T(1,2);
    x_upper_right=center1+T(1,3);
    x_upper_left=center1+T(1,4);
    y_lower_left=center2+T(2,1);
    y_lower_right=center2+T(2,2);
    y_upper_right=center2+T(2,3);
    y_upper_left=center2+T(2,4);
    x_coor=[x_lower_left x_lower_right x_upper_right x_upper_left];
    y_coor=[y_lower_left y_lower_right y_upper_right y_upper_left];
    %axis equal;
%     rectangle('Position',[agent_pos(1)-agent_rad agent_pos(2)-agent_rad agent_rad*2 agent_rad*2],'Curvature',[1,1]); %use for circular agent
    patch('Vertices',[x_coor; y_coor]','Faces',[1 2 3 4],'Edgecolor','blue','Facecolor','none','Linewidth',1.2);% use this for rectangular agent
    hold on;
    if (has_obstacle)
            for k = 1:length(obst_rad)
%         rectangle('Position',[obst_pos(1)-obst_rad obst_pos(2)-obst_rad obst_rad*2 obst_rad*2],'Curvature',[1,1]);
                R= ([cos(obst_theta(k)), -sin(obst_theta(k)); sin(obst_theta(k)), cos(obst_theta(k))]);
                L = obst_rad(k);
                H = obst_rad(k);
                X=([-L/2, L/2, L/2, -L/2]);
                Y=([-H/2, -H/2, H/2, H/2]);
                for i=1:4
                    T(:,i)=R*[X(i); Y(i)];
                end
                center1 = obst_pos(k,1);
                center2 = obst_pos(k,2);
                x_lower_left=center1+T(1,1);
                x_lower_right=center1+T(1,2);
                x_upper_right=center1+T(1,3);
                x_upper_left=center1+T(1,4);
                y_lower_left=center2+T(2,1);
                y_lower_right=center2+T(2,2);
                y_upper_right=center2+T(2,3);
                y_upper_left=center2+T(2,4);
                x_coor=[x_lower_left x_lower_right x_upper_right x_upper_left];
                y_coor=[y_lower_left y_lower_right y_upper_right y_upper_left];
                patch('Vertices',[x_coor; y_coor]','Faces',[1 2 3 4],'Edgecolor','red','Facecolor','none','Linewidth',1.2);% use this for rectangular agent
                hold on;
            end
    end
    f = plot(agent_goal(1),agent_goal(2),'rx');
    set(gcf,'Position',[600 600 900 900]);
end