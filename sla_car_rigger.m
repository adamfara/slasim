clear; clc; close all;

%% Define some unit conversion constants
m2mm = 1000;
mm2in = 1/25.4;
m2in = m2mm * mm2in;


%% Constant transform used to make plotting in MATLAB more natural
%  upside down z axis wasn't playing nicely with plot3 / view
R = [1 0 0;  0 -1 0;  0 0 -1];

n_step_ride = 20;
n_step_steering = 20;
desired_ride_travel = 3;
desired_steering_travel = 1.8;
n_points = 10000;

name = ['sla-t' num2str(desired_ride_travel) '-s' num2str(n_step_ride) '-n' num2str(n_points) '-s' num2str(n_step_steering)];


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
             0.2913857422, 0.0424136989, -0.3498214906; %shock inboard
             0.0000000000, 0.6424709459, -0.2627653100  %wheel spindle ref 
             ].*m2in;

frontright = [1.6438880000, 0.2692400000, -0.3395980000; %ufibj
              1.5212060000, 0.5332567693, -0.3900867017;
              1.3462000000, 0.2692400000, -0.3395980000;
              1.6438880000, 0.2047240000, -0.1210564000; %lfibj
              1.5440152000, 0.5661661950, -0.1223983744;
              1.3385683592, 0.2047240000, -0.1210564000;
              1.5367000000, 0.5787389021, -0.2597158005; %wc
              1.5367000000, 0.5969000000,  0.0000000000; %wcp 
              1.5938500000, 0.2206625000, -0.1460754000; %tri
              1.5979080055, 0.5741775550, -0.1591240731; %tro
              1.5397081584, 0.5314113662, -0.1483045150; %pro
              1.4378560390, 0.2461155929, -0.3651473833; %bc pivot
              1.4809566053, 0.2461155929, -0.3853919229; %bc axis
              1.4011630229, 0.3146125350, -0.4432667085; %bc shock
              1.4378560390, 0.2020414055, -0.3651473833; %bc arb
              1.4193016907, 0.2993868857, -0.4046495371; %bc pushrod
              1.3906638194, 0.1849437500, -0.5043048932; %arb link pt
              1.4478000000, 0.1936750000, -0.5309479318; %arb pivot
              1.4478000000, 0.0000000000, -0.5309479318; %arb axis
              1.3558372496, 0.1802775981, -0.5397651490; %shock inboard
              1.5367000000, 0.6523112000, -0.2648604770  %wheel spindle ref 
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
% Rears have no steer effect.  To evaluate 4-wheel steer need to change
% this
rrr = sla_kinematics(rearright, -1, desired_ride_travel, n_step_ride, n_points, carpos);
rrj = sla_kinematics(rearright, 1, desired_ride_travel, n_step_ride, n_points, carpos);
rr_lut = cat(3, rrr(:,:,end:-1:2), rrj);

rl_lut = rr_lut;
rl_lut(:,2,:) = -rl_lut(:,2,:);

% Front wheels are steered at the TRI
step_steering = desired_steering_travel / (n_step_steering - 1);
fr_lut = zeros([size(rr_lut) 2*n_step_steering-1]);
fl_lut = zeros([size(rr_lut) 2*n_step_steering-1]);

frr = sla_kinematics(frontright, -1, desired_ride_travel, n_step_ride, n_points, carpos);
frj = sla_kinematics(frontright, 1, desired_ride_travel, n_step_ride, n_points, carpos);
fr_lut(:,:,:,n_step_steering) = cat(3, frr(:,:,end:-1:2), frj);
frontright_steered = frontright;

flr = sla_kinematics(frontleft, -1, desired_ride_travel, n_step_ride, n_points, carpos);
flj = sla_kinematics(frontleft, 1, desired_ride_travel, n_step_ride, n_points, carpos);
fl_lut(:,:,:,n_step_steering) = cat(3, flr(:,:,end:-1:2), flj);
frontleft_steered = frontleft;


for ii = 2:n_step_steering
    frontright_steered = sla_steer(frontright_steered, step_steering, n_points);
    frontleft_steered = sla_steer(frontleft_steered, step_steering, n_points);
    
    frr = sla_kinematics(frontright_steered, -1, desired_ride_travel, n_step_ride, n_points, carpos);
    frj = sla_kinematics(frontright_steered, 1, desired_ride_travel, n_step_ride, n_points, carpos);
    fr_lut(:,:,:,ii + n_step_steering - 1) = cat(3, frr(:,:,end:-1:2), frj);
    
    flr = sla_kinematics(frontleft_steered, -1, desired_ride_travel, n_step_ride, n_points, carpos);
    flj = sla_kinematics(frontleft_steered, 1, desired_ride_travel, n_step_ride, n_points, carpos);
    fl_lut(:,:,:,ii + n_step_steering - 1) = cat(3, flr(:,:,end:-1:2), flj);
end

frontright_steered = frontright;
frontleft_steered = frontleft;
for ii = 2:n_step_steering
    frontright_steered = sla_steer(frontright_steered, -step_steering, n_points);
    frontleft_steered = sla_steer(frontleft_steered, -step_steering, n_points);
    
    frr = sla_kinematics(frontright_steered, -1, desired_ride_travel, n_step_ride, n_points, carpos);
    frj = sla_kinematics(frontright_steered, 1, desired_ride_travel, n_step_ride, n_points, carpos);
    fr_lut(:,:,:,-ii+1+n_step_steering) = cat(3, frr(:,:,end:-1:2), frj);
    
    flr = sla_kinematics(frontleft_steered, -1, desired_ride_travel, n_step_ride, n_points, carpos);
    flj = sla_kinematics(frontleft_steered, 1, desired_ride_travel, n_step_ride, n_points, carpos);
    fl_lut(:,:,:,-ii+1+n_step_steering) = cat(3, flr(:,:,end:-1:2), flj);
end

% fl_lut = fr_lut;
% fl_lut(:,2,:,:) = -fl_lut(:,2,:,end:-1:1);


%% Calculate geometric parameters for the rig
rr_geo = sla_geometry(rr_lut);
rl_geo = sla_geometry(rl_lut);

fr_geo = sla_geometry(fr_lut);
fl_geo = sla_geometry(fl_lut);


%% Save outputs for future use
sla.rearright = rearright;
sla.rearleft = rearleft;
sla.frontright = frontright;
sla.frontleft = frontleft;
sla.rr = rr_lut;
sla.rl = rl_lut;
sla.fr = fr_lut;
sla.fl = fl_lut;
sla.rr_geo = rr_geo;
sla.rl_geo = rl_geo;
sla.fr_geo = fr_geo;
sla.fl_geo = fl_geo;
sla.carpos = carpos;
sla.carbox = carbox;
sla.n_step_ride = n_step_ride;
sla.n_step_steering = n_step_steering;
sla.n_points = n_points;
sla.desired_ride_travel = desired_ride_travel;
sla.desired_steering_travel = desired_steering_travel;

clearvars -except sla name n_step_ride n_step_steering
save([name '.mat']);


%% Actuate suspension through ride range

plotting = false;

if (plotting)
    figure(1); clf; hold on;
    hs.o = PER_plot_origin(sla.carbox, sla.carpos);

    hs.rr_o = PER_plot_SLA(sla.rearright,   sla.carpos);
    hs.rl_o = PER_plot_SLA(sla.rearleft,    sla.carpos);
    hs.fr_o = PER_plot_SLA(sla.frontright,  sla.carpos);
    hs.fl_o = PER_plot_SLA(sla.frontleft,   sla.carpos);

    hs.rr = PER_plot_SLA(sla.rearright,     sla.carpos, 0);
    hs.rl = PER_plot_SLA(sla.rearleft,      sla.carpos, 0);
    hs.fr = PER_plot_SLA(sla.frontright,    sla.carpos, 0);
    hs.fl = PER_plot_SLA(sla.frontleft,     sla.carpos, 0);

    drawnow;
    
    while(plotting)
        for ii = [(2*sla.n_step_ride - 1):-1:2 1:(2*sla.n_step_ride - 1)-1]
        %     disp(ii);
            PER_plot_SLA(sla.rr(:,:,ii), sla.carpos, 0, hs.rr);
            PER_plot_SLA(sla.rl(:,:,ii), sla.carpos, 0, hs.rl);
            PER_plot_SLA(sla.fr(:,:,ii,n_step_steering), sla.carpos, 0, hs.fr);
            PER_plot_SLA(sla.fl(:,:,ii,n_step_steering), sla.carpos, 0, hs.fl);

            drawnow;
        end
    end
end


%% Actuate suspension through steered range

plotting = true;

if (plotting)
    figure(1); clf; hold on;
    hs.o = PER_plot_origin(sla.carbox, sla.carpos);
    view(-90,90);

    hs.rr_o = PER_plot_SLA(sla.rearright,   sla.carpos);
    hs.rl_o = PER_plot_SLA(sla.rearleft,    sla.carpos);
    hs.fr_o = PER_plot_SLA(sla.frontright,  sla.carpos);
    hs.fl_o = PER_plot_SLA(sla.frontleft,   sla.carpos);

    hs.rr = PER_plot_SLA(sla.rearright,     sla.carpos, 0);
    hs.rl = PER_plot_SLA(sla.rearleft,      sla.carpos, 0);
    hs.fr = PER_plot_SLA(sla.frontright,    sla.carpos, 0);
    hs.fl = PER_plot_SLA(sla.frontleft,     sla.carpos, 0);

    drawnow;
    
    while(plotting)
        for ii = [(2*sla.n_step_steering - 1):-1:2 1:(2*sla.n_step_steering - 1)-1]
%             disp(ii);
            PER_plot_SLA(sla.rr(:,:,n_step_ride), sla.carpos, 0, hs.rr);
            PER_plot_SLA(sla.rl(:,:,n_step_ride), sla.carpos, 0, hs.rl);
            PER_plot_SLA(sla.fr(:,:,n_step_ride,ii), sla.carpos, 0, hs.fr);
            PER_plot_SLA(sla.fl(:,:,n_step_ride,ii), sla.carpos, 0, hs.fl);

            drawnow;
        end
    end
end