%Suspension member load calculator
%Foster Collins
%Fall 2015

%Requires a loaded look up table

function [loads] = link_loads(F, M, ShockDisplacement, SteeringAngle, CarLocation, sla)
%calculates forces in suspension members based on forces and moments at the
%contact patch
%steering angle for current wheel **not steering wheel**

%returns loads which is a column vector with the loads in N in the 6
%members

%[forward upper AA link;
% rearward upper AA link;
% steering/toe link;
% forward bottom AA link;
% rearward bottom AA link;
% pushrod];

switch CarLocation
    case char('FrontRight')
        %determine closest shock travel point
        [~, indexShock] = min(abs(ShockDisplacement-sla.fr_geo.shock_travel));
        [~, indexSteer] = min(abs(SteeringAngle-sla.fr_geo.steered_angle(:,indexShock)));
        
        %construct vectors for each link
        v1 = sla.fr(1,:,indexShock, indexSteer)-sla.fr(3,:,indexShock, indexSteer); %ufibj-uobj
        v2 = sla.fr(2,:,indexShock, indexSteer)-sla.fr(3,:,indexShock, indexSteer); %uribj-uobj
        v3 = sla.fr(9,:,indexShock, indexSteer)-sla.fr(10,:,indexShock, indexSteer); %tri-tro
        v4 = sla.fr(4,:,indexShock, indexSteer)-sla.fr(6,:,indexShock, indexSteer); %lfibj-lobj
        v5 = sla.fr(5,:,indexShock, indexSteer)-sla.fr(6,:,indexShock, indexSteer); %lribj-lobj
        v6 = sla.fr(16,:,indexShock, indexSteer) - sla.fr(11,:,indexShock, indexSteer); %bc pushrod - pro 

        %vector from contact patch to where the force is applied
        p1 = sla.fr(3,:,indexShock, indexSteer)-sla.fr(7,:,indexShock, indexSteer); %uobj
        p2 = sla.fr(3,:,indexShock, indexSteer)-sla.fr(7,:,indexShock, indexSteer); %uobj
        p3 = sla.fr(10,:,indexShock, indexSteer)-sla.fr(7,:,indexShock, indexSteer); %tro
        p4 = sla.fr(6,:,indexShock, indexSteer)-sla.fr(7,:,indexShock, indexSteer); %lobj
        p5 = sla.fr(6,:,indexShock, indexSteer)-sla.fr(7,:,indexShock, indexSteer); %lobj
        p6 = sla.fr(11,:,indexShock, indexSteer)-sla.fr(7,:,indexShock, indexSteer); %pro

    case char('FrontLeft')
        [~, indexShock] = min(abs(ShockDisplacement-sla.fl_geo.shock_travel));
        [~, indexSteer] = min(abs(SteeringAngle-sla.fl_geo.steered_angle(:,indexShock)));
end

%normalize vecotor
u1 = v1./norm(v1);
u2 = v2./norm(v2);
u3 = v3./norm(v3);
u4 = v4./norm(v4);
u5 = v5./norm(v5);
u6 = v6./norm(v6);

A = [u1', u2', u3', u4', u5', u6';
      cross(p1,u1)' cross(p2,u2)' cross(p3,u3)' cross(p4,u4)' cross(p5,u5)' cross(p6,u6)'];
  
B  = [F';M'];

loads = inv(A)*B;

end

