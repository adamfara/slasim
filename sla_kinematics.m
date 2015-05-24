%%
% Adam Farabaugh
% Penn Electric Racing
% SLA suspension rigger
%
% Input inboard points and outboard points at ride height
% Solve for point locations at some deflection
%
% Actuation is ignored, i.e. pushrod/bellcrank location not known
% These can be integrated later through motion ratio math
%
% Coordinate system is SAE J670 - Z down, X forward, Y outboard
% Assume we do this for the right side, can flip for left

function [ lookup ] = sla_kinematics( ipts, direction, travel, n_step, n_points, carpos )
% ipts:
% uaf - U A ARM, Front BJ
% uao - U A ARM, Outer BJ
% uar - U A ARM, Rear BJ
% laf - L A ARM, Front BJ
% lao - L A ARM, Outer BJ
% lar - L A ARM, Rear BJ
% whc - Wheel Center
% wcp - Wheel Contact Patch Point
% tri - Tie Rod, Inner BJ
% tro - Tie Rod, Outer BJ
% pro
% bc pivot
% bc axis
% bc shock
% bc arb
% bc pr
% arb link pt
% arb pivot
% arb axis
% shock inboard

upper_fibj = ipts(1,:);
upper_ribj = ipts(3,:);
upper_obj = ipts(2,:);          % changes with travel

lower_fibj = ipts(4,:);
lower_ribj = ipts(6,:);
lower_obj = ipts(5,:);          % changes with travel

wheel_center_init = ipts(7,:);  % changes with travel
contact_patch_init = ipts(8,:); % changes with travel

tierod_inner = ipts(9,:);
tierod_outer = ipts(10,:);      % changes with travel

pushrod_outer_init = ipts(11,:);     % changes with travel

bellcrank_pivot = ipts(12,:);
bellcrank_axis = ipts(13,:);
bellcrank_shock_init = ipts(14,:);   % changes with travel
bellcrank_arb_init = ipts(15,:);     % changes with travel
bellcrank_pushrod_init = ipts(16,:); % changes with travel
arb_link_pt_init = ipts(17,:);       % changes with travel
arb_pivot = ipts(18,:);
arb_axis = ipts(19,:);
shock_inboard = ipts(20,:);     


%% get closest point on upper a-arm axis to outer bj by projection
g = upper_fibj - upper_ribj;
g_unit = g/(sqrt(sum(g.^2)));
v = upper_obj - upper_ribj;
cp_upper_offset = dot(g_unit,v);
cp_upper = upper_ribj + cp_upper_offset*g_unit;

upper_vector = upper_obj - cp_upper;
upper_length = norm(upper_vector, 2);


%% get closest point on lower a-arm axis to outer bj by projection
u = lower_fibj - lower_ribj;
u_unit = u/(sqrt(sum(u.^2)));
v = lower_obj - lower_ribj;
cp_lower_offset = dot(u_unit,v);
cp_lower = lower_ribj + cp_lower_offset*u_unit;

lower_vector = lower_obj - cp_lower;
lower_length = norm(lower_vector, 2);


%%
step = -direction*(travel/2) / n_step;


%% fixed distance between outer ball joints
w = upper_obj - lower_obj;
dist_obj = norm(w,2);


%% Numerically construct a circle around the A-arms
theta = linspace(0, 2*pi, n_points);
% theta2 = linspace(-pi/4, pi/4, n_points);
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

bc_len_pr = bellcrank_pushrod - bellcrank_pivot;
bc_circ_pr = bsxfun(@plus, bellcrank_pivot', bc_xprime*cos(theta)*bc_len_pr + bc_yprime*sin(theta)*bc_len_pr)';


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
    
    
    %Find the intersection of the circle described by the upper A-arm OBJ
    %with the sphere defined by the distance between the lower A-arm OBJ at
    %its center and a radius equal to the length between the ball joints.
    %Since this will generally have two solutions, the index is found for
    %both cases.
    err2 = bsxfun(@minus, sqrt(sum(bsxfun(@minus, upper_circ, P_3D(i,:)).^2,2)), dist_obj);
    mins = find(imregionalmin(abs(err2)));
    mins(mins == 1) = [];
    mins(mins == n_points) = [];
    
    min_index_inc = mins(1);
    min_index_dec = mins(2);
    
    %In order to find the 'correct' intersection, the program looks to the
    %last solution found. Whichever intersection point is closer to the
    %last solution will be the correct one.
    point_inc = upper_circ(min_index_inc,:);
    point_dec = upper_circ(min_index_dec,:);
    dist_inc = norm((point_inc - Q_3D(i-1,:)),2);
    dist_dec = norm((point_dec - Q_3D(i-1,:)),2);
    if dist_inc < dist_dec
        Q_3D(i,:) = point_inc;
    else
        Q_3D(i,:) = point_dec;
    end
%     disp(min(abs(err2(min_index_inc)), abs(err2(min_index_dec))));

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
    
    %Construct the circle numerically (i.e., generate n_points on the
    %circle)
    circle = bsxfun(@plus, close_point(i,:), (e*cos(theta)*r_circ + f*sin(theta)*r_circ)');
    dist_err = bsxfun(@minus, sqrt(sum(bsxfun(@minus, circle, tierod_inner).^2, 2)), ltierod);
    
    %Finding indices of minimum dist_err. Since the intersection of a
    %sphere and circle is generally two points, there will be two minima
    
    mins = find(imregionalmin(abs(dist_err)));
    mins(mins == 1) = [];
    mins(mins == n_points) = [];
    min_index_inc = mins(1);
    min_index_dec = mins(2);
    
    %Determine which of the two solutions is correct by checking which
    %defines a point closer to the last value
    tobj_inc = circle(min_index_inc(1),:,1);
    tobj_dec = circle(min_index_dec(1),:,1);
    dist_inc = norm((tobj_inc - tierod_obj(i-1,:)),2);
    dist_dec = norm((tobj_dec - tierod_obj(i-1,:)),2);
    if dist_inc < dist_dec
        tierod_obj(i,:) = circle(min_index_inc,:);
    else
        tierod_obj(i,:) = circle(min_index_dec,:);
    end
    
    %Next, use transformation matrices to find the location of the wheel
    %center and contact patch. Since the wheel center and contact patch are
    %fixed w.r.t. the steering knuckle, the transformation matrix along
    %with the orientation of the knuckle uniquely describe the contact
    %patch and wheel center locations.
    
    o = close_point(i,:);
    y = Q_3D(i,:)-o;
    
    y = y./sqrt(sum(y.^2));
    z = tierod_obj(i,:)-o;
    z = z./sqrt(sum(z.^2));
    x = cross(y,z);
    T_knuckle = zeros(4);
    T_knuckle(1:3,1) = x';
    T_knuckle(1:3,2) = y';
    T_knuckle(1:3,3) = z';
    T_knuckle(1:3,4) = o';
    T_knuckle(4,4) = 1;
    
    if i == 2
        
        %The first time through, need to find the position of the wheel center
        %and contact patch w.r.t. the knuckle coordinate system. Since this
        %does not change, it only needs to be calculated once.
        o_first = first_cp;
        y_first = Q_3D(1,:) - o_first;
        y_first = y_first./sqrt(sum(y_first.^2));
        z_first = tierod_obj(i,:) - o_first;
        z_first = z_first./sqrt(sum(z_first.^2));
        x_first = cross(y_first,z_first);
        T_knuckle_first = zeros(4);
        T_knuckle_first(1:3,1) = x_first';
        T_knuckle_first(1:3,2) = y_first';
        T_knuckle_first(1:3,3) = z_first';
        T_knuckle_first(1:3,4) = o_first';
        T_knuckle_first(4,4) = 1;
        
        wheel_center_init(4) = 1;
        wc_kcs = T_knuckle_first\wheel_center_init';
        wheel_center(1,:) = wheel_center_init(1:3);
        
        contact_patch_init(4) = 1;
        cp_kcs = T_knuckle_first\contact_patch_init';
        contact_patch(1,:) = contact_patch_init(1:3);
        
        % Also need to get the constant transformation of the lower a-arm
        % used to calculate the location of the pushrod obj
        o_first = lower_obj;
        y_first = lower_ribj - o_first;
        y_first = y_first./sqrt(sum(y_first.^2));
        z_first = lower_fibj - o_first;
        z_first = z_first./sqrt(sum(z_first.^2));
        x_first = cross(y_first,z_first);
        % now recorrect z because y and z are not perpendicular to start
        % with
        z_first = cross(x_first, y_first);
        % build transformation
        T_la_first = zeros(4);
        T_la_first(1:3,1) = x_first';
        T_la_first(1:3,2) = y_first';
        T_la_first(1:3,3) = z_first';
        T_la_first(1:3,4) = o_first';
        T_la_first(4,4) = 1;
        
        la_kcs = T_la_first \ [pushrod_outer 1]';
        
    end
    
    %Solve for wheel center positions
    t4 = T_knuckle * wc_kcs;
    wheel_center(i,:) = t4(1:3)';
    
    %Solve for contact patch positions
    t5 = T_knuckle * cp_kcs;
    contact_patch(i,:) = t5(1:3)';
    
end

%% Exporting lookup table
    lookup = zeros(20, 3, n_step);
    
%     keyboard;
    lookup(1,:,:)  = repmat(upper_fibj, [1 1 n_step]);
    lookup(2,:,:)  = reshape(Q_3D', [1 3 n_step]);
    lookup(3,:,:)  = repmat(upper_ribj, [1 1 n_step]);
    lookup(4,:,:)  = repmat(lower_fibj, [1 1 n_step]);
    lookup(5,:,:)  = reshape(P_3D', [1 3 n_step]);
    lookup(6,:,:)  = repmat(lower_ribj, [1 1 n_step]);
    lookup(7,:,:)  = reshape(wheel_center', [1 3 n_step]);
    lookup(8,:,:)  = reshape(contact_patch', [1 3 n_step]);
    lookup(9,:,:)  = repmat(tierod_inner, [1 1 n_step]);
    lookup(10,:,:) = reshape(tierod_obj', [1 3 n_step]);
    
    lookup(11,:,:) = reshape(pushrod_outer', [1 3 n_step]);
    lookup(12,:,:) = repmat(bellcrank_pivot, [1 1 n_step]);
    lookup(13,:,:) = repmat(bellcrank_axis, [1 1 n_step]);
    lookup(14,:,:) = reshape(bellcrank_shock', [1 3 n_step]);
    lookup(15,:,:) = reshape(bellcrank_arb', [1 3 n_step]);
    lookup(16,:,:) = reshape(bellcrank_pr', [1 3 n_step]);
    lookup(17,:,:) = reshape(arb_link_pt', [1 3 n_step]);
    lookup(18,:,:) = repmat(arb_pivot, [1 1 n_step]);
    lookup(19,:,:) = repmat(arb_axis, [1 1 n_step]);
    lookup(20,:,:) = repmat(shock_inboard, [1 1 n_step]);

end

