%%
% Adam Farabaugh
% Penn Electric Racing
% SLA suspension rigger
%
% Input inboard points and outboard points at ride height
% Solve for point locations at some deflection
%
% Coordinate system is SAE J670 - Z down, X forward, Y outboard
% Assume we do this for the right side, can flip for left

function [ lookup ] = sla_kinematics( ipts, direction, travel, n_step, n_points, carpos )
% ipts:
% uaf uao uar
% laf lao lar
% wc wcp
% tri tro
% pro
% bc pivot  bc axis  bc shock  bc arb  bc pr
% arb link pt  arb pivot  arb axis
% shock inboard

upper_fibj = ipts(1,:);
upper_ribj = ipts(3,:);
upper_obj = ipts(2,:);              % changes with travel

lower_fibj = ipts(4,:);
lower_ribj = ipts(6,:);
lower_obj = ipts(5,:);              % changes with travel

wheel_center_init = ipts(7,:);      % changes with travel
contact_patch_init = ipts(8,:);     % changes with travel

tierod_inner = ipts(9,:);
tierod_outer = ipts(10,:);          % changes with travel

pushrod_outer_init = ipts(11,:);    % changes with travel

bellcrank_pivot = ipts(12,:);
bellcrank_axis = ipts(13,:);
bellcrank_shock_init = ipts(14,:);  % changes with travel
bellcrank_arb_init = ipts(15,:);    % changes with travel
bellcrank_pushrod_init = ipts(16,:);% changes with travel
arb_link_pt_init = ipts(17,:);      % changes with travel
arb_pivot = ipts(18,:);
arb_axis = ipts(19,:);
shock_inboard = ipts(20,:);
spindle_ref_init = ipts(21,:);      % changes with travel


%% get closest point on upper a-arm axis to outer bj by projection
g = upper_fibj - upper_ribj;
g_unit = g / norm(g);
v = upper_obj - upper_ribj;
cp_upper_offset = dot(g_unit, v);
cp_upper = upper_ribj + cp_upper_offset * g_unit;

upper_vector = upper_obj - cp_upper;
upper_length = norm(upper_vector);


%% get closest point on lower a-arm axis to outer bj by projection
u = lower_fibj - lower_ribj;
u_unit = u / norm(u);
v = lower_obj - lower_ribj;
cp_lower_offset = dot(u_unit, v);
cp_lower = lower_ribj + cp_lower_offset * u_unit;

lower_vector = lower_obj - cp_lower;
lower_length = norm(lower_vector);


%% get closest point on arb axis to arb link point
r = arb_pivot - arb_axis;
r_unit = r ./ norm(r);
v = arb_link_pt_init - arb_pivot;
cp_arb_offset = dot(r_unit, v);
cp_arb = arb_pivot + cp_arb_offset * r_unit;


%% Step size discretization
step = -direction*(travel/2) / n_step;


%% Fixed distance between outer ball joints
w = upper_obj - lower_obj;
dist_obj = norm(w,2);


%% Numerically construct a circle around the A-arms
theta = linspace(0, 2*pi, n_points);
t1 = null(u_unit);
t2 = null(g_unit);
a = t1(:,1);
b = t1(:,2);
c = t2(:,1);
d = t2(:,2);

lower_circ = bsxfun(@plus, cp_lower', a*cos(theta)*lower_length + b*sin(theta)*lower_length)';
upper_circ = bsxfun(@plus, cp_upper', c*cos(theta)*upper_length + d*sin(theta)*upper_length)';

H = [carpos.R carpos.t; 0 0 0 1];
% cput = H*[cp_upper'; 1];
% cplt = H*[cp_lower'; 1];
% plot3(cput(1),cput(2),cput(3),'g*')
% plot3(cplt(1),cplt(2),cplt(3),'r*')

% lct = H*[lower_circ'; ones(1,size(lower_circ,1))];
% uct = H*[upper_circ'; ones(1,size(upper_circ,1))];
% plot3(uct(1,:),uct(2,:),uct(3,:),'g')
% plot3(lct(1,:),lct(2,:),lct(3,:),'r')


%% Numerically construct a circle around bellcrank axis for pr
bc = bellcrank_pivot - bellcrank_axis;
bc_unit = bc ./ norm(bc);
tbc = null(bc_unit);
bc_xprime = tbc(:,1);
bc_yprime = tbc(:,2);

bc_len_pr = norm(bellcrank_pushrod_init - bellcrank_pivot);
bc_circ_pr = bsxfun(@plus, bellcrank_pivot', bc_xprime*cos(theta)*bc_len_pr + bc_yprime*sin(theta)*bc_len_pr)';
pushrod_length = norm(bellcrank_pushrod_init - pushrod_outer_init);


%% Numerically construct a circle around ARB axis for arb link pt
arb = cp_arb - arb_axis;
arb_unit = arb ./ norm(arb);
tarb = null(arb_unit);
arb_xprime = tarb(:,1);
arb_yprime = tarb(:,2);

arb_arm = norm(cp_arb - arb_link_pt_init);
arb_circ = bsxfun(@plus, cp_arb', arb_xprime*cos(theta)*arb_arm + arb_yprime*sin(theta)*arb_arm)';
droplink_len = norm(bellcrank_arb_init - arb_link_pt_init);

%% Upper OBJ
Q_3D = zeros(n_step,3);
Q_3D(1,:) = upper_obj;


%% Lower OBJ
P_3D = zeros(n_step,3);
P_3D(1,:) = lower_obj;
desired_pos = zeros(1,3);


%Finding where along the steering knuckle normal vector is closest to the
%tierod outer ball joint
close_point = zeros(n_step,3);
j = upper_obj - lower_obj;
j_unit = j./norm(j);
h = tierod_outer-lower_obj;
close_point_offset = dot(j_unit,h);
first_cp = P_3D(1,:) + close_point_offset*j_unit;
close_point(1,:) = first_cp;
r_circ = norm(tierod_outer - first_cp);
ltierod = norm(tierod_outer-tierod_inner);


%Set up variables
tierod_obj = zeros(n_step,3);
tierod_obj(1,:) = tierod_outer;
wheel_center = zeros(n_step,3);
contact_patch = zeros(n_step,3);
spindle_ref = zeros(n_step,3);
pushrod_outer = zeros(n_step,3);
bellcrank_shock = zeros(n_step,3);
bellcrank_arb = zeros(n_step,3);
bellcrank_pr = zeros(n_step,3);
arb_link_pt = zeros(n_step,3);

for i = 2:n_step
    %Calculate desired position (Y value is the important one)
    desired_pos(1) = P_3D((i-1),1);
    desired_pos(2) = P_3D((i-1),2);
    desired_pos(3) = P_3D((i-1),3) - step;
    
    %Find point on lower_circ closest to desired position
    err_x = abs(desired_pos(1) - lower_circ(:,1));
    err_y = abs(desired_pos(2) - lower_circ(:,2));
    err_z = abs(desired_pos(3) - lower_circ(:,3));
    
    [~, index] = min(sqrt(sum([err_x err_y err_z].^2, 2)));

    %Lower OBJ location is the point with the lowest distance error
    P_3D(i,:) = lower_circ(index,:);
    
    %Search through the discretizated circle for Upper OBJ location 
    [Qtemp, minQerr] = find_intersection_circle_sphere( upper_circ, P_3D(i,:), Q_3D(i-1,:), dist_obj);
    Q_3D(i,:) = Qtemp;
%     disp(minQerr);

    %Finding tie rod outer ball joint location (i.e., coupler rotation)
    %Normalized (unit) vector of steering knuckle
    q = Q_3D(i,:) - P_3D(i,:);
    unit = q./sqrt(sum(q.^2));
    
    %Closest point on knuckle normal vector to the tierod obj
    close_point(i,:) = P_3D(i,:) + close_point_offset*unit;
    
    %Find two vectors orthogonal to 'unit' (and each other) in order to
    %construct the circle
    t3 = null(unit);
    e = t3(:,1);
    f = t3(:,2);
    
    circle = bsxfun(@plus, close_point(i,:), (e*cos(theta)*r_circ + f*sin(theta)*r_circ)');
    [tobjtemp, minTRerr] = find_intersection_circle_sphere(circle, tierod_inner, tierod_obj(i-1,:), ltierod);
    tierod_obj(i,:) = tobjtemp;
%     disp(minTRerr);
    
    if i == 2
        %The first time through, need to find the position of the wheel center
        %and contact patch w.r.t. the knuckle coordinate system. Since this
        %does not change, it only needs to be calculated once.
        wc_kcs = get_kcs(first_cp, Q_3D(1,:), tierod_obj(i,:), wheel_center_init);
        cp_kcs = get_kcs(first_cp, Q_3D(1,:), tierod_obj(i,:), contact_patch_init);
        sr_kcs = get_kcs(first_cp, Q_3D(1,:), tierod_obj(i,:), spindle_ref_init);
        
        wheel_center(1,:) = wheel_center_init;
        contact_patch(1,:) = contact_patch_init;
        spindle_ref(1,:) = spindle_ref_init;
        
        % Also need to get the constant transformation of the lower a-arm
        % used to calculate the location of the pushrod obj
        la_kcs = get_kcs(lower_obj, lower_ribj, lower_fibj, pushrod_outer_init);
        
        pushrod_outer(1,:) = pushrod_outer_init;
        
        % Also get bellcrank KCSs for shock and arb attachment.  Pushrod
        % attachment to bellcrank will be driven by a numerical search and
        % is the reference point for the y-axis.
        bc_kcs_sh = get_kcs(bellcrank_pivot, bellcrank_pushrod_init, bellcrank_axis, bellcrank_shock_init);
        bc_kcs_arb = get_kcs(bellcrank_pivot, bellcrank_pushrod_init, bellcrank_axis, bellcrank_arb_init);
        
        bellcrank_shock(1,:) = bellcrank_shock_init;
        bellcrank_arb(1,:) = bellcrank_arb_init;
        bellcrank_pr(1,:) = bellcrank_pushrod_init;
        arb_link_pt(1,:) = arb_link_pt_init;
    end
    
    %Solve for wheel center positions
    wheel_center(i,:) = apply_kcs(wc_kcs, close_point(i,:), Q_3D(i,:), tierod_obj(i,:));
    
    %Solve for contact patch positions
    contact_patch(i,:) = apply_kcs(cp_kcs, close_point(i,:), Q_3D(i,:), tierod_obj(i,:));
    
    %Solve for spindle ref point positions
    spindle_ref(i,:) = apply_kcs(sr_kcs, close_point(i,:), Q_3D(i,:), tierod_obj(i,:));
    
    %Solve for pushrod obj positions
    pushrod_outer(i,:) = apply_kcs(la_kcs, P_3D(i,:), lower_ribj, lower_fibj);
    
    
    % Find bellcrank orientation with pr length
    [bcprtemp, minBCPRerr] = find_intersection_circle_sphere(bc_circ_pr, pushrod_outer(i,:), bellcrank_pr(i-1,:), pushrod_length);
    bellcrank_pr(i,:) = bcprtemp;
%     disp(minBCPRerr);
    
    %Solve for bellcrank shock pt position
    bellcrank_shock(i,:) = apply_kcs(bc_kcs_sh, bellcrank_pivot, bellcrank_pr(i,:), bellcrank_axis);
    
    %Solve for bellcrank arb pt position
    bellcrank_arb(i,:) = apply_kcs(bc_kcs_arb, bellcrank_pivot, bellcrank_pr(i,:), bellcrank_axis);
    
    % Find arb rocker orientation with droplink length
%     keyboard;
    [arbtemp, minARBerr] = find_intersection_circle_sphere(arb_circ, bellcrank_arb(i,:), arb_link_pt(i-1,:), droplink_len);
    arb_link_pt(i,:) = arbtemp;
%     disp(minARBerr);
    
end

%% Exporting lookup table
    lookup = zeros(21, 3, n_step);
    
%     keyboard;
    lookup(1,:,:)  = repmat(upper_fibj, [1 1 n_step]);
    lookup(2,:,:)  = reshape(Q_3D', [1 3 n_step]);%
    lookup(3,:,:)  = repmat(upper_ribj, [1 1 n_step]);
    lookup(4,:,:)  = repmat(lower_fibj, [1 1 n_step]);
    lookup(5,:,:)  = reshape(P_3D', [1 3 n_step]);%
    lookup(6,:,:)  = repmat(lower_ribj, [1 1 n_step]);
    lookup(7,:,:)  = reshape(wheel_center', [1 3 n_step]);%
    lookup(8,:,:)  = reshape(contact_patch', [1 3 n_step]);%
    lookup(9,:,:)  = repmat(tierod_inner, [1 1 n_step]);
    lookup(10,:,:) = reshape(tierod_obj', [1 3 n_step]);%
    
    lookup(11,:,:) = reshape(pushrod_outer', [1 3 n_step]);%
    lookup(12,:,:) = repmat(bellcrank_pivot, [1 1 n_step]);
    lookup(13,:,:) = repmat(bellcrank_axis, [1 1 n_step]);
    lookup(14,:,:) = reshape(bellcrank_shock', [1 3 n_step]);%
    lookup(15,:,:) = reshape(bellcrank_arb', [1 3 n_step]);%
    lookup(16,:,:) = reshape(bellcrank_pr', [1 3 n_step]);%
    lookup(17,:,:) = reshape(arb_link_pt', [1 3 n_step]);
    lookup(18,:,:) = repmat(arb_pivot, [1 1 n_step]);
    lookup(19,:,:) = repmat(arb_axis, [1 1 n_step]);
    lookup(20,:,:) = repmat(shock_inboard, [1 1 n_step]);
    lookup(21,:,:) = reshape(spindle_ref', [1 3 n_step]);%

end

