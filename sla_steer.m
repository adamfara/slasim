%% 
% Adam Farabaugh
% Penn Electric Racing
% SLA suspension rigger

function [ opts ] = sla_steer( ipts, step, n_points )
    % ipts:
    % uaf uao uar
    % laf lao lar
    % wc wcp
    % tri tro
    % pro
    % bc pivot  bc axis  bc shock  bc arb  bc pr
    % arb link pt  arb pivot  arb axis
    % shock inboard
    
    %% Read LUT
%     keyboard
    upper_fibj          = squeeze(ipts(1,:,:));
    upper_ribj          = squeeze(ipts(3,:,:));
    upper_obj           = squeeze(ipts(2,:,:));
    lower_fibj          = squeeze(ipts(4,:,:));
    lower_ribj          = squeeze(ipts(6,:,:));
    lower_obj           = squeeze(ipts(5,:,:));
    wheel_center        = squeeze(ipts(7,:,:));
    contact_patch       = squeeze(ipts(8,:,:));
    tierod_inner        = squeeze(ipts(9,:,:));
    tierod_outer        = squeeze(ipts(10,:,:));
    pushrod_outer       = squeeze(ipts(11,:,:));
    bellcrank_pivot     = squeeze(ipts(12,:,:));
    bellcrank_axis      = squeeze(ipts(13,:,:));
    bellcrank_shock     = squeeze(ipts(14,:,:));
    bellcrank_arb       = squeeze(ipts(15,:,:));
    bellcrank_pushrod   = squeeze(ipts(16,:,:));
    arb_link_pt         = squeeze(ipts(17,:,:));
    arb_pivot           = squeeze(ipts(18,:,:));
    arb_axis            = squeeze(ipts(19,:,:));
    shock_inboard       = squeeze(ipts(20,:,:));
    spindle_ref         = squeeze(ipts(21,:,:));
    
    theta = linspace(0, 2*pi, n_points);

    tierod_len = norm(tierod_outer - tierod_inner);
    
    steer_axis = upper_obj - lower_obj;
    steer_axis_unit = steer_axis ./ norm(steer_axis);
    tsa = null(steer_axis_unit);
    sa_xprime = tsa(:,1);
    sa_yprime = tsa(:,2);
    
    h = tierod_outer - lower_obj;
    close_point_offset = dot(steer_axis_unit, h);
    cp = lower_obj + close_point_offset * steer_axis_unit;
    steer_arm_radius = norm(tierod_outer - cp);
    
    sa_circle = bsxfun(@plus, cp', sa_xprime*cos(theta)*steer_arm_radius + sa_yprime*sin(theta)*steer_arm_radius)';
    
    % move the steering rack, assuming it's aligned with the y axis
    tierod_inner(2) = tierod_inner(2) + step;
    
    % find the new outer tierod location
    [totemp, mintoerr] = find_intersection_circle_sphere(sa_circle, tierod_inner, tierod_outer, tierod_len);
    
    % we need to get kinematic coordinate systems to all the important
    % unsprung points before we actually move the tierod outer ball joint
    wc_kcs = get_kcs(cp, upper_obj, tierod_outer, wheel_center);
    cp_kcs = get_kcs(cp, upper_obj, tierod_outer, contact_patch);
    sr_kcs = get_kcs(cp, upper_obj, tierod_outer, spindle_ref);
    
    % move the outboard assembly
    tierod_outer = totemp;
    
    % now apply the above KCS calculations to find new locations of
    % everything
    wheel_center  = apply_kcs(wc_kcs, cp, upper_obj, tierod_outer);
    contact_patch = apply_kcs(cp_kcs, cp, upper_obj, tierod_outer);
    spindle_ref   = apply_kcs(sr_kcs, cp, upper_obj, tierod_outer);
    
    % write the new locations to output
    opts = [
    upper_fibj;
    upper_obj;
    upper_ribj;
    lower_fibj;
    lower_obj;
    lower_ribj;
    wheel_center;
    contact_patch;
    tierod_inner;
    tierod_outer;
    pushrod_outer;
    bellcrank_pivot;
    bellcrank_axis;
    bellcrank_shock;
    bellcrank_arb;
    bellcrank_pushrod;
    arb_link_pt;
    arb_pivot;
    arb_axis;
    shock_inboard;
    spindle_ref];
    
%     opts = ipts;
    
end