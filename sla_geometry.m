%% 
% Adam Farabaugh
% Penn Electric Racing
% SLA suspension rigger

function [ opts ] = sla_geometry( ipts )

    n_steps_ride = size(ipts, 3);
    n_steps_steering = size(ipts, 4);
    
    opts.camber             = zeros(n_steps_steering, n_steps_ride);
    opts.steered_angle      = zeros(n_steps_steering, n_steps_ride);
    opts.caster             = zeros(n_steps_steering, n_steps_ride);
    opts.trail              = zeros(n_steps_steering, n_steps_ride);
    opts.kingpin            = zeros(n_steps_steering, n_steps_ride);
    opts.scrub              = zeros(n_steps_steering, n_steps_ride);
    opts.spindle            = zeros(n_steps_steering, n_steps_ride);
    opts.fv_ic              = zeros(n_steps_steering, 3, n_steps_ride);
%     opts.sv_ic              = zeros(n_steps_steering, n_steps_ride);
%     opts.rch                = zeros(n_steps_steering, n_steps_ride);
%     opts.camber_change_rate = zeros(n_steps_steering, n_steps_ride);

    for ii = 1:n_steps_steering
    
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
    upper_fibj          = squeeze(ipts(1,:,:,ii));
    upper_ribj          = squeeze(ipts(3,:,:,ii));
    upper_obj           = squeeze(ipts(2,:,:,ii));
    lower_fibj          = squeeze(ipts(4,:,:,ii));
    lower_ribj          = squeeze(ipts(6,:,:,ii));
    lower_obj           = squeeze(ipts(5,:,:,ii));
    wheel_center        = squeeze(ipts(7,:,:,ii));
    contact_patch       = squeeze(ipts(8,:,:,ii));
    tierod_inner        = squeeze(ipts(9,:,:,ii));
    tierod_outer        = squeeze(ipts(10,:,:,ii));
    pushrod_outer       = squeeze(ipts(11,:,:,ii));
    bellcrank_pivot     = squeeze(ipts(12,:,:,ii));
    bellcrank_axis      = squeeze(ipts(13,:,:,ii));
    bellcrank_shock     = squeeze(ipts(14,:,:,ii));
    bellcrank_arb       = squeeze(ipts(15,:,:,ii));
    bellcrank_pushrod   = squeeze(ipts(16,:,:,ii));
    arb_link_pt         = squeeze(ipts(17,:,:,ii));
    arb_pivot           = squeeze(ipts(18,:,:,ii));
    arb_axis            = squeeze(ipts(19,:,:,ii));
    shock_inboard       = squeeze(ipts(20,:,:,ii));
    spindle_ref         = squeeze(ipts(21,:,:,ii));
    
    
    %% Camber Curves
    % Find by angle between (a) vector from CP normal to ground (xy
    % plane) and (b) vector from CP to WC
    camber_a = contact_patch - [contact_patch(1:2,:); 10*ones(1,size(contact_patch,2))];
    camber_b = wheel_center - contact_patch;
    
    camber_a = bsxfun(@rdivide, camber_a, sqrt(sum(camber_a.^2,1)));
    camber_b = bsxfun(@rdivide, camber_b, sqrt(sum(camber_b.^2,1)));
    
    camber = -acosd(dot(camber_a, camber_b));
    
    
    %% Toe / Steered Angle
    toe_a = contact_patch - wheel_center;
    toe_b = spindle_ref - wheel_center;
    
    toe_vec = cross(toe_a, toe_b);
    toe_vec = bsxfun(@rdivide, toe_vec, sqrt(sum(toe_vec.^2,1)));
    toe = wrapTo360(atan2d(toe_vec(2,:), toe_vec(1,:))) - 180;
    
    
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
    
    fv_ic = [zeros(1, n_steps_ride);
             -(dot_lower .* -upper_normal(3, :) - dot_upper .* -lower_normal(3, :)) ./ instant_axis_normal(1,:);
             -(dot_upper .* -lower_normal(2, :) - dot_lower .* -upper_normal(2, :)) ./ instant_axis_normal(1,:)];
    
    % Side View Instant Center
%     t = (contact_patch(1) - fv_ic(1))/instant_axis_normal(1);
%     sv_ic(1) = contact_patch(1);
%     sv_ic(2) = fv_ic(2) + t*instant_axis_normal(2);
%     sv_ic(3) = fv_ic(3) + t*instant_axis_normal(3);
    
    
    % Camber Change Rate
    
    
    
    %% Calculate some interesting travels - can correlate to data and such
    opts.cp_travel = contact_patch(3,:);
    opts.shock_travel = sqrt(sum((bellcrank_shock - shock_inboard).^2,1));
    opts.shock_travel = opts.shock_travel - opts.shock_travel((n_steps_ride+1)/2);
    
    % can only do arb angle relative to centerline - need other corner for
    % full bar displacement angle
    r = arb_pivot(:,(n_steps_ride+1)/2) - arb_axis(:,(n_steps_ride+1)/2);
    r_unit = r ./ norm(r);
    v = arb_link_pt(:,(n_steps_ride+1)/2) - arb_pivot(:,(n_steps_ride+1)/2);
    cp_arb_offset = dot(r_unit, v);
    cp_arb = arb_pivot(:,(n_steps_ride+1)/2) + cp_arb_offset * r_unit;
    
    arb_a = bsxfun(@minus, arb_link_pt, cp_arb);
    arb_b = repmat(arb_a(:,(n_steps_ride+1)/2), 1, n_steps_ride);
    
    arb_a = bsxfun(@rdivide, arb_a, sqrt(sum(arb_a.^2,1)));
    arb_b = bsxfun(@rdivide, arb_b, sqrt(sum(arb_b.^2,1)));
    
    % Here we let negative ride travel (jounce due to SAE coord system) is
    % negative arb angular displacement.  This should not really matter as
    % long as you always use this, because restoring moment is proportional
    % to the magnitude of the sum of displacements (torsional spring
    % assumed)
    opts.arb_ang_disp = sign(opts.cp_travel) .* acosd(dot(arb_a, arb_b));
    
    %% Collect other outputs
    opts.camber(ii,:) = camber;
    opts.steered_angle(ii,:) = toe;
    opts.caster(ii,:) = caster;
    opts.trail(ii,:) = trail;
    opts.kingpin(ii,:) = kingpin;
    opts.scrub(ii,:) = scrub;
    opts.spindle(ii,:) = spindle;
    opts.fv_ic(ii,:,:) = fv_ic;
%     opts.sv_ic(ii,:) = sv_ic;
%     opts.rch(ii,:) = RCH;
%     opts.camber_change_rate(ii,:) = camber_change_rate;

    end

    
end