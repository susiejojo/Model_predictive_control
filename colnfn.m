function [c,ceq] = colnfn(u,obst_pos,has_lane_con,has_obstacle,radii_sum,agent_goal,agent_pos,theta,time_sample,pred_horizon,obst_controls,obst_theta)
    c = [];
    lane_dists = [];
    if (has_obstacle)
        for i= 1:length(obst_theta)
            [~,dists] = nonhn_pts(u,agent_pos,agent_goal,theta,time_sample,pred_horizon);
            lane_dists = dists;
            [~,obst_dists] = nonhn_pts([obst_controls(1:pred_horizon,i),obst_controls(pred_horizon+1:2*pred_horizon,i)],obst_pos(i,:),agent_goal,obst_theta(i),time_sample,pred_horizon);
    %         obst_net = [obst_net obst_dists];
%             obst_dists
            dists = (dists(:,1) - obst_dists(:,1)).^2 + (dists(:,2) - obst_dists(:,2)).^2;
            if ((agent_pos(1)-obst_pos(i,1))<5 && (agent_pos(2)-obst_pos(i,2))<5)
              c = [c;-dists + (radii_sum)^2+7];
            end
        end
    end
%     dists = (dists(:,1) - obst_dists(:,1)).^2 + (dists(:,2) - obst_dists(:,2)).^2;
    %for lane constraints
    if (has_lane_con)
        for i= 1:pred_horizon
            c = [c; -(lane_dists(i,1)-1-lane_dists(i,2)+26)]; %left lane y = x + 25
            c = [c; (lane_dists(i,1)-1-lane_dists(i,2)-24)]; %right lane y = x - 25
        end
    end
    ceq = [];
end