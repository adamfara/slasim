function [ hps ] = PER_plot_origin( carbox, carpos, varargin )
    vx = [6 0 0 1]';
    vy = [0 4 0 1]';
    vz = [0 0 3 1]';
    vo = [0 0 0 1]';
    
    H = [carpos.R carpos.t; 0 0 0 1];
    
    to = H*vo;
    tx = H*vx;
    ty = H*vy;
    tz = H*vz;
    
    hold on;
    
    if (nargin <= 2)
        hx = plot3([to(1) tx(1)], [to(2) tx(2)], [to(3) tx(3)], '-r');
        hy = plot3([to(1) ty(1)], [to(2) ty(2)], [to(3) ty(3)], '-g');
        hz = plot3([to(1) tz(1)], [to(2) tz(2)], [to(3) tz(3)], '-b');
        hps = [hx; hy; hz];
    else
        set(varargin{1}(1), 'XData', [to(1) tx(1)], 'YData', [to(2) tx(2)], 'ZData', [to(3) tx(3)]);
        set(varargin{1}(2), 'XData', [to(1) ty(1)], 'YData', [to(2) ty(2)], 'ZData', [to(3) ty(3)]);
        set(varargin{1}(3), 'XData', [to(1) tz(1)], 'YData', [to(2) tz(2)], 'ZData', [to(3) tz(3)]);
        hps = varargin{1};
    end
    axis equal;
    axis(carbox);
    view(-30, 20);
    
end

