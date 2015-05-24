clear; clc; close all;

%% Define some unit conversion constants
m2mm = 1000;
mm2in = 1/25.4;
m2in = m2mm * mm2in;


%% Constant transform used to make plotting in MATLAB more natural
%  upside down z axis wasn't playing nicely with plot3 / view
R = [1 0 0;  0 -1 0;  0 0 -1];


%% Define static ride-height points
rearright = [0.2794000000, 0.2195195000, -0.3278632000; %ufibj
             0.0361950000, 0.5447740356, -0.3831778871; %uribj
             0.0381000000, 0.2195195000, -0.3278632000; %uobj
             0.2794000000, 0.1912493000, -0.1397969211; %lfibj
             0.0000000000, 0.5711045339, -0.1240779215; %lribj
             0.0381000000, 0.1912493000, -0.1113997211; %lobj
             0.0000000000, 0.5687639160, -0.2601914018; %wc
             0.0000000000, 0.5778500000,  0.0000000000; %wcp
             0.0381000000, 0.2195195000, -0.3278632000; %tri
            -0.0361950000, 0.5447740356, -0.3831778871; %tro
             0.0143541788, 0.5249209977, -0.1492816144; %pro
             0.1978266417, 0.2053660655, -0.3531266595; %bc pivot
             0.2391223852, 0.2290760164, -0.3531266595; %bc axis
             0.2140847200, 0.1770492869, -0.4226916518; %bc shock
             0.1832501068, 0.2307540913, -0.3531266595; %bc arb
             0.1865864235, 0.2249432113, -0.3860997910; %bc pushrod
             0.2036572000, 0.2071687500, -0.1135724155; %arb link pt
             0.2667000000, 0.2159000000, -0.1135724155; %arb pivot
             0.2667000000, 0.0000000000, -0.1135724155; %arb axis
             0.2913857422, 0.0424136989, -0.3498214906  %shock inboard
             ].*m2in;

frontright = [1.6438880000, 0.2692400000, -0.3395980000;
              1.5212060000, 0.5332567693, -0.3900867017;
              1.3462000000, 0.2692400000, -0.3395980000;
              1.6438880000, 0.2047240000, -0.1210564000;
              1.5440152000, 0.5661661950, -0.1223983744;
              1.3385683592, 0.2047240000, -0.1210564000;
              1.5367000000, 0.5787389021, -0.2597158005;
              1.5367000000, 0.5969000000, 0.0000000000;
              1.5938500000, 0.2206625000, -0.1460754000;
              1.5979080055, 0.5741775550, -0.1591240731;
              1.5397081584, 0.5314113662, -0.1483045150;
              1.5397081584, 0.5314113662, -0.1483045150; %pro
              1.4378560390, 0.2461155929, -0.3651473833; %bc pivot
              1.4809566053, 0.2461155929, -0.3853919229; %bc axis
              1.4011630229, 0.3146125350, -0.4432667085; %bc shock
              1.4378560390, 0.2020414055, -0.3651473833; %bc arb
              1.4193016907, 0.2993868857, -0.4046495371; %bc pushrod
              1.3906638194, 0.1849437500, -0.5043048932; %arb link pt
              1.4478000000, 0.1936750000, -0.5309479318; %arb pivot
              1.4478000000, 0.0000000000, -0.5309479318; %arb axis
              1.3558372496, 0.1802775981, -0.5397651490  %shock inboard
              ].*m2in;
              
% can flip the y-coord to get the other half of the car
rearleft = rearright;
rearleft(:,2) = rearleft(:,2) .* -1;
frontleft = frontright;
frontleft(:,2) = frontleft(:,2) .* -1;


%% Set up plot of full car
carbox = [-12 85 -30 30 -4 44];
carpos.R = eye(3)*R;
carpos.t = zeros(3,1);


%% Rig suspensions and generate lookup tables for travel
n_step = 20;
desired_travel = 4;
n_points = 10000;

rrr = sla_kinematics(rearright, -1, desired_travel, n_step, n_points, carpos);
rrj = sla_kinematics(rearright, 1, desired_travel, n_step, n_points, carpos);
rr_lut = cat(3, rrr(:,:,end:-1:2), rrj);
rl_lut = rr_lut;
rl_lut(:,2,:) = -rl_lut(:,2,:);

frr = sla_kinematics(frontright, -1, desired_travel, n_step, n_points, carpos);
frj = sla_kinematics(frontright, 1, desired_travel, n_step, n_points, carpos);
fr_lut = cat(3, frr(:,:,end:-1:2), frj);
fl_lut = fr_lut;
fl_lut(:,2,:) = -fl_lut(:,2,:);


%% Test that it works by actuating suspension through range
figure(1); clf; hold on;
hs.o = PER_plot_origin(carbox, carpos);

hs.rr_o = PER_plot_SLA(rearright, carpos);
hs.rl_o = PER_plot_SLA(rearleft, carpos);
hs.fr_o = PER_plot_SLA(frontright, carpos);
hs.fl_o = PER_plot_SLA(frontleft, carpos);

hs.rr = PER_plot_SLA(rearright, carpos, 0);
hs.rl = PER_plot_SLA(rearleft, carpos, 0);
hs.fr = PER_plot_SLA(frontright, carpos, 0);
hs.fl = PER_plot_SLA(frontleft, carpos, 0);

drawnow;

for ii = 1:(2*n_step - 1)
%     disp(ii);
    PER_plot_SLA(rr_lut(:,:,ii), carpos, 0, hs.rr);
    PER_plot_SLA(rl_lut(:,:,ii), carpos, 0, hs.rl);
    PER_plot_SLA(fr_lut(:,:,ii), carpos, 0, hs.fr);
    PER_plot_SLA(fl_lut(:,:,ii), carpos, 0, hs.fl);

    drawnow;
end
    
while(true)%false)%
    for ii = (2*n_step - 1):-1:1
    %     disp(ii);
        PER_plot_SLA(rr_lut(:,:,ii), carpos, 0, hs.rr);
        PER_plot_SLA(rl_lut(:,:,ii), carpos, 0, hs.rl);
        PER_plot_SLA(fr_lut(:,:,ii), carpos, 0, hs.fr);
        PER_plot_SLA(fl_lut(:,:,ii), carpos, 0, hs.fl);

        drawnow;
    end
    for ii = 1:(2*n_step - 1)
    %     disp(ii);
        PER_plot_SLA(rr_lut(:,:,ii), carpos, 0, hs.rr);
        PER_plot_SLA(rl_lut(:,:,ii), carpos, 0, hs.rl);
        PER_plot_SLA(fr_lut(:,:,ii), carpos, 0, hs.fr);
        PER_plot_SLA(fl_lut(:,:,ii), carpos, 0, hs.fl);

        drawnow;
    end
end

%% Calculate geometric parameters for the rig