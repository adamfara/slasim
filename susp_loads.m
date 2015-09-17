clear; clc; close all;

%%
load('sla-t3-s100-n100000.mat');

car.sla = sla;
car.ftrack = 47;
car.rtrack = 45.5;
car.wheelbase = 60.5;
car.fwd = 46.5 / 100;
car.hcog = 11;
car.mass = 450 + 150;
car.mass_unsprung = 24.5;
car.mass_sprung = car.mass - 4*car.mass_unsprung;

car.wr_static = (1 - car.fwd) * car.mass;
car.wf_static = car.fwd * car.mass;

car.wrr_static = car.wr_static / 2;
car.wrl_static = car.wr_static / 2;
car.wfr_static = car.wf_static / 2;
car.wfl_static = car.wf_static / 2;