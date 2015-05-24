function [ hps ] = PER_plot_SLA( pts, carpos, varargin  )

    H = [carpos.R carpos.t; 0 0 0 1];
    pts = H*[pts, ones(size(pts,1),1)]';
    
    if (nargin <= 3)
        if nargin == 2
            c1 = [.9 .9 .9];
            c2 = [.7 .7 .7];
        elseif nargin == 3
            c1 = 'b';
            c2 = 'r';
        end
        hps.hpul = plot3(pts(1,1:3), pts(2,1:3), pts(3,1:3), '-', 'Color', c1);
        hps.hpum = plot3(pts(1,1:3), pts(2,1:3), pts(3,1:3), 'o', 'MarkerEdgeColor', c2);

        hps.hpll = plot3(pts(1,4:6), pts(2,4:6), pts(3,4:6), '-', 'Color', c1);
        hps.hplm = plot3(pts(1,4:6), pts(2,4:6), pts(3,4:6), 'o', 'MarkerEdgeColor', c2);

        hps.hpwl = plot3(pts(1,7:8), pts(2,7:8), pts(3,7:8), '-', 'Color', c1);
        hps.hpwm = plot3(pts(1,7:8), pts(2,7:8), pts(3,7:8), 'o', 'MarkerEdgeColor', c2);

        hps.hptl = plot3(pts(1,9:10), pts(2,9:10), pts(3,9:10), '-', 'Color', c1);
        hps.hptm = plot3(pts(1,9:10), pts(2,9:10), pts(3,9:10), 'o', 'MarkerEdgeColor', c2);

        hps.hpurl = plot3(pts(1,[2 5 10 2]), pts(2,[2 5 10 2]), pts(3,[2 5 10 2]), '-', 'Color', c1);
        
        hps.hpprl = plot3(pts(1,[11 16]), pts(2,[11 16]), pts(3,[11 16]), '-', 'Color', c1);
        hps.hpprm = plot3(pts(1,[11 16]), pts(2,[11 16]), pts(3,[11 16]), 'o', 'MarkerEdgeColor', c2);
        
        hps.hpbcl = plot3(pts(1,[12 14 12 15 12 16]), pts(2,[12 14 12 15 12 16]), pts(3,[12 14 12 15 12 16]), '-', 'Color', c1);
        
        hps.hparb = plot3(pts(1,17:19), pts(2,17:19), pts(3,17:19), '-', 'Color', c1);
        
        hps.hpdll = plot3(pts(1,[15 17]), pts(2,[15 17]), pts(3,[15 17]), '-', 'Color', c1);
        hps.hpdlm = plot3(pts(1,[15 17]), pts(2,[15 17]), pts(3,[15 17]), 'o', 'MarkerEdgeColor', c2);
        
        hps.hpsl = plot3(pts(1,[14 20]), pts(2,[14 20]), pts(3,[14 20]), '-', 'Color', c1);
        hps.hpsm = plot3(pts(1,[14 20]), pts(2,[14 20]), pts(3,[14 20]), 'o', 'MarkerEdgeColor', c2);
    else
        set(varargin{2}.hpul, 'XData', pts(1,1:3), 'YData', pts(2,1:3), 'ZData', pts(3,1:3));
        set(varargin{2}.hpum, 'XData', pts(1,1:3), 'YData', pts(2,1:3), 'ZData', pts(3,1:3));
        
        set(varargin{2}.hpll, 'XData', pts(1,4:6), 'YData', pts(2,4:6), 'ZData', pts(3,4:6));
        set(varargin{2}.hplm, 'XData', pts(1,4:6), 'YData', pts(2,4:6), 'ZData', pts(3,4:6));
        
        set(varargin{2}.hptl, 'XData', pts(1,7:8), 'YData', pts(2,7:8), 'ZData', pts(3,7:8));
        set(varargin{2}.hptm, 'XData', pts(1,7:8), 'YData', pts(2,7:8), 'ZData', pts(3,7:8));
        
        set(varargin{2}.hpwl, 'XData', pts(1,9:10), 'YData', pts(2,9:10), 'ZData', pts(3,9:10));
        set(varargin{2}.hpwm, 'XData', pts(1,9:10), 'YData', pts(2,9:10), 'ZData', pts(3,9:10));
        
        set(varargin{2}.hpurl, 'XData', pts(1,[2 5 10 2]), 'YData', pts(2,[2 5 10 2]), 'ZData', pts(3,[2 5 10 2]));
        
        set(varargin{2}.hpprl, 'XData', pts(1,[11 16]), 'YData', pts(2,[11 16]), 'ZData', pts(3,[11 16]));
        set(varargin{2}.hpprm, 'XData', pts(1,[11 16]), 'YData', pts(2,[11 16]), 'ZData', pts(3,[11 16]));
        
        set(varargin{2}.hpbcl, 'XData', pts(1,[12 14 12 15 12 16]), 'YData', pts(2,[12 14 12 15 12 16]), 'ZData', pts(3,[12 14 12 15 12 16]));
        
        set(varargin{2}.hparb, 'XData', pts(1,17:19), 'YData', pts(2,17:19), 'ZData', pts(3,17:19));
        
        set(varargin{2}.hpdll, 'XData', pts(1,[15 17]), 'YData', pts(2,[15 17]), 'ZData', pts(3,[15 17]));
        set(varargin{2}.hpdlm, 'XData', pts(1,[15 17]), 'YData', pts(2,[15 17]), 'ZData', pts(3,[15 17]));
        
        set(varargin{2}.hpsl, 'XData', pts(1,[14 20]), 'YData', pts(2,[14 20]), 'ZData', pts(3,[14 20]));
        set(varargin{2}.hpsm, 'XData', pts(1,[14 20]), 'YData', pts(2,[14 20]), 'ZData', pts(3,[14 20]));
        
        hps = varargin{2};
    end
end


