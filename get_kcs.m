function [ kcs ] = get_kcs( rigid_body_origin, rigid_body_y, rigid_body_z_prime, rigid_body_interest_point )
%GET_KCS Gets a kinematic coordinate system attached to the rigid body
% defined by an origin, a point on the y axis, and another point which
% is planar with the y-z plane.  
% Expects 1x3 vectors as input for points - these are rows.

    o = rigid_body_origin;
    
    y = rigid_body_y - o;
    y = y ./ norm(y);
    
    zp = rigid_body_z_prime - o;
    zp = zp ./ norm(zp);
    
    x = cross(y, zp);
    z = cross(x, y);
    
    T = [x y z o; 0 0 0 1];
    kcs = T \ rigid_body_interest_point;

end

