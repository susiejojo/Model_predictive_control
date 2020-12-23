function [c,ceq] = colnfn(u,obst_pos,has_lane_con,has_obstacle,radii_sum,agent_goal,agent_pos,theta,time_sample,pred_horizon,obst_controls,obst_theta)
    c = [];
    [~,dists] = nonhn_pts(u,agent_pos,agent_goal,theta,time_sample,pred_horizon);
    [~,obst_dists] = nonhn_pts(obst_controls,obst_pos,agent_goal,obst_theta,time_sample,pred_horizon);
    lane_dists = dists;
    dists = (dists(:,1) - obst_dists(:,1)).^2 + (dists(:,2) - obst_dists(:,2)).^2+2.5;
    if (has_obstacle)
        if ((agent_pos(1)-obst_pos(1))<5 && (agent_pos(2)-obst_pos(2))<5)
            c = [c;-dists + (radii_sum)^2];
        end
    end
    %for lane constraints
    if (has_lane_con)
        for i= 1:pred_horizon
            c = [c; -(lane_dists(i,1)-1-lane_dists(i,2)+26)]; %left lane y = x + 25
            c = [c; (lane_dists(i,1)-1-lane_dists(i,2)-24)]; %right lane y = x - 25
        end
    end
    ceq = [];
end