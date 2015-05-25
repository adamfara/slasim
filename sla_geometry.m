%% 
% Adam Farabaugh
% Penn Electric Racing
% SLA suspension rigger

function [ opts ] = sla_geometry( ipts )
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
    arb_link_pt_init    = squeeze(ipts(17,:,:));
    arb_pivot           = squeeze(ipts(18,:,:));
    arb_axis            = squeeze(ipts(19,:,:));
    shock_inboard       = squeeze(ipts(20,:,:));
    
    n_steps = size(ipts, 3);

    
    
    
    %% Camber Curves
    % Find by angle between (a) vector from CP normal to ground (xy
    % plane) and (b) vector from CP to WC
    camber_a = contact_patch - [contact_patch(1:2,:); 10*ones(1,size(contact_patch,2))];
    camber_b = wheel_center - contact_patch;
    
    camber_a = bsxfun(@rdivide, camber_a, sqrt(sum(camber_a.^2,1)));
    camber_b = bsxfun(@rdivide, camber_b, sqrt(sum(camber_b.^2,1)));
    
    camber = -acosd(dot(camber_a, camber_b));
    
    
    %% Caster Angles and Mechanical Trail
    caster_z = -upper_obj(3,:) + lower_obj(3,:);
    caster_x = lower_obj(1,:) - upper_obj(1,:);
    caster = atand(caster_x ./ caster_z);
    
    m_caster = -caster_z ./ caster_x;
    b_caster = -upper_obj(3,:) - m_caster .* upper_obj(1,:);
    trail = -b_caster ./ m_caster;
    
    
    %% Kingpin Inclination, Spindle Length, and Scrub Radius
    kp_x = -upper_obj(2,:) + lower_obj(2,:);
    kp_y = -upper_obj(3,:) + lower_obj(3,:);
    kingpin = atand(kp_x ./ kp_y);
    
    m_kp = kp_y ./ kp_x;
    b_kp = -upper_obj(3,:) + m_kp.*upper_obj(2,:);
    scrub = -contact_patch(2,:) - (-b_kp ./ m_kp);
    spindle = -wheel_center(2,:) - (-wheel_center(3,:) - b_kp) ./ m_kp;
    
    
    %% Instant Centers
    % Get A-arm plane normal vectors    
    upper_normal = cross( (upper_fibj - upper_ribj), (upper_obj - upper_ribj) );
    lower_normal = cross( (lower_fibj - lower_ribj), (lower_obj - lower_ribj) );
    
    upper_normal = bsxfun(@rdivide, upper_normal, sqrt(sum(upper_normal.^2, 1)));
    lower_normal = bsxfun(@rdivide, lower_normal, sqrt(sum(lower_normal.^2, 1)));
    
    % Find normal vector of instant axis
    instant_axis_normal = cross(upper_normal, lower_normal);
    
    % Find point corresponding to front view instant center
    dot_upper = -dot(upper_normal, upper_fibj);
    dot_lower = -dot(lower_normal, lower_fibj);
    
    fv_ic = zeros(3,1);
    
    fv_ic = [zeros(1, n_steps);
             -(dot_lower .* -upper_normal(3, :) - dot_upper .* -lower_normal(3, :)) ./ instant_axis_normal(1,:);
             (dot_upper .* -lower_normal(2, :) - dot_lower .* -upper_normal(2, :)) ./ instant_axis_normal(1,:)];
         
    keyboard;
    
    % Side View Instant Center
    %% Camber Change Rate
    
    
    
    
    
    
    
    
    
    
    
end