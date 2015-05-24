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
        hpul = plot3(pts(1,1:3), pts(2,1:3), pts(3,1:3), '-', 'Color', c1);
        hpum = plot3(pts(1,1:3), pts(2,1:3), pts(3,1:3), 'o', 'MarkerEdgeColor', c2);

        hpll = plot3(pts(1,4:6), pts(2,4:6), pts(3,4:6), '-', 'Color', c1);
        hplm = plot3(pts(1,4:6), pts(2,4:6), pts(3,4:6), 'o', 'MarkerEdgeColor', c2);

        hpwl = plot3(pts(1,7:8), pts(2,7:8), pts(3,7:8), '-', 'Color', c1);
        hpwm = plot3(pts(1,7:8), pts(2,7:8), pts(3,7:8), '*', 'MarkerEdgeColor', c2);

        hptl = plot3(pts(1,9:10), pts(2,9:10), pts(3,9:10), '-', 'Color', c1);
        hptm = plot3(pts(1,9:10), pts(2,9:10), pts(3,9:10), 'o', 'MarkerEdgeColor', c2);

        hpurl = plot3(pts(1,[2 5 10 2]), pts(2,[2 5 10 2]), pts(3,[2 5 10 2]), '-', 'Color', c1);
    
        hps = [hpul; hpum; hpll; hplm; hptl; hptm; hpwl; hpwm; hpurl];
    else
        set(varargin{2}(1), 'XData', pts(1,1:3), 'YData', pts(2,1:3), 'ZData', pts(3,1:3));
        set(varargin{2}(2), 'XData', pts(1,1:3), 'YData', pts(2,1:3), 'ZData', pts(3,1:3));
        
        set(varargin{2}(3), 'XData', pts(1,4:6), 'YData', pts(2,4:6), 'ZData', pts(3,4:6));
        set(varargin{2}(4), 'XData', pts(1,4:6), 'YData', pts(2,4:6), 'ZData', pts(3,4:6));
        
        set(varargin{2}(5), 'XData', pts(1,7:8), 'YData', pts(2,7:8), 'ZData', pts(3,7:8));
        set(varargin{2}(6), 'XData', pts(1,7:8), 'YData', pts(2,7:8), 'ZData', pts(3,7:8));
        
        set(varargin{2}(7), 'XData', pts(1,9:10), 'YData', pts(2,9:10), 'ZData', pts(3,9:10));
        set(varargin{2}(8), 'XData', pts(1,9:10), 'YData', pts(2,9:10), 'ZData', pts(3,9:10));
        
        set(varargin{2}(9), 'XData', pts(1,[2 5 10 2]), 'YData', pts(2,[2 5 10 2]), 'ZData', pts(3,[2 5 10 2]));
        
        hps = varargin{2};
    end
end

