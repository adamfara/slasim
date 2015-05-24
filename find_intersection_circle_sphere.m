function [ p, min_err ] = find_intersection_circle_sphere( circle_pts, ref_pt, prev_pt, dist_goal )
    err = bsxfun(@minus, sqrt(sum(bsxfun(@minus, circle_pts, ref_pt).^2,2)), dist_goal);
    mins = find(imregionalmin(abs(err)));
    mins(mins == 1) = [];
    mins(mins == length(circle_pts)) = [];
    
    min_index_inc = mins(1);
    min_index_dec = mins(2);
    
    %In order to find the 'correct' intersection, the program looks to the
    %last solution found. Whichever intersection point is closer to the
    %last solution will be the correct one.
    point_inc = circle_pts(min_index_inc,:);
    point_dec = circle_pts(min_index_dec,:);
    dist_inc = norm(point_inc - prev_pt);
    dist_dec = norm(point_dec - prev_pt);
    if dist_inc < dist_dec
        p = point_inc;
        min_err = abs(err(min_index_inc));
    else
        p = point_dec;
        min_err = abs(err(min_index_dec));
    end
end

