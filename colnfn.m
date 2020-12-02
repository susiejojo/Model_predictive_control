function [c,ceq] = colnfn(u,obst_pos,radii_sum,agent_pos,theta,time_sample,pred_horizon)
    [~,dists] = nonhn_pts(u,agent_pos,theta,time_sample,pred_horizon);
%     fprintf("dists: %d",size(dists));
%     size(dists)
%     size(u)
%     size(dists)
    dists = (dists(:,1) - obst_pos(1)).^2 + (dists(:,2) - obst_pos(2)).^2;
    c = -dists + (radii_sum+0.5)^2;
    ceq = [];
end