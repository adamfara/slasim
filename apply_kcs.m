function [ p ] = apply_kcs( kcs, rigid_body_origin, rigid_body_y, rigid_body_z_prime )
%APPLY_KCS Uses generated KCS along with reference points (same as used to
%generate the KCS!) to find the current position of a point attached to a
%rigid body.
% Expects 1x3 vectors as inputs.  Outputs 1x3 vector.

    o = rigid_body_origin;
    
    y = rigid_body_y - o;
    y = y ./ norm(y);
    
    zp = rigid_body_z_prime - o;
    zp = zp ./ norm(zp);
    
    x = cross(y, zp);
    z = cross(x, y);
    
    T = [x y z o; 0 0 0 1];
    p_homogeneous = T * kcs;
    p = p_homogeneous(1:3)';

end

