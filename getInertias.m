function [plane] = getInertias(plane)

% x(1)  %ft, wing lcg location
% x(2)              %fuselage cg location  
% x(3)    %tail cg location
% x(4)    %tail cg location
% x(5)    %landing gear cg location
% x(6)    %engine cg location
% x(7)    %fuel systems cg location
% x(8)    %surface controls cg location
% x(9)     %avionics cg location
% x(10)   %fuel cg location
% %11th element is payload, at predrop cg location

wing = plane.geo.wing;
body = plane.geo.body;
h_tail = plane.geo.h_tail;
v_tail = plane.geo.v_tail;
nacelle = plane.geo.nacelle;
x = plane.data.weight.x;
z = plane.data.weight.z;
cgx = plane.data.weight.CGx;
cgz = plane.data.weight.CGz;

W = [plane.data.weight.W(1:11), plane.data.weight.W(1:11), plane.data.weight.W(1:11)];
W(10,2:3) = plane.data.weight.fuel_2;
W(11,3) = 0;
W = W/32.174049; % FROM HERE ON OUT W IS MASS ARRAY

x = [x-cgx(1), x-cgx(2), x-cgx(3)];
z = [z-cgz(1), z-cgz(2), z-cgz(3)];

%%%%%%%%%%%% EVERYTHING CONSIDERED AS POINT MASS, rectangle, cylinder, or cylindrical shell%%%%%%%%%%%%%%%%
%% Ixx
Ixx_self = zeros(11,3);

Ixx_self(1,:) = (1/12)*W(1,:)*(wing.b^2); % doesn't take TR into account
Ixx_self(2,:) = (W(2,:)*(body.D/2)^2);
Ixx_self(3,:) = (1/12)*W(3,:)*(h_tail.b^2);
Ixx_self(4,:) = (1/12)*W(4,:)*(v_tail.b^2);
Ixx_self(5,:) = 0;
Ixx_self(6,:) = 0.5*W(6,:)*(nacelle.D/2)^2;
Ixx_self(7,:) = 0;
Ixx_self(8,:) = 0;
Ixx_self(9,:) = 0;
Ixx_self(10,:) = (1/12)*W(10,:)*((wing.b/2)^2); %assume fuel box half wing span
Ixx_self(11,:) = 0.5*W(11,:)*(body.D/2)^2;

Ixx_all = Ixx_self + W.*(z.^2);
Ixx = sum(Ixx_all,1);

%% Iyy
Iyy_self = zeros(11,3);

Iyy_self(1,:) = (1/12)*W(1,:)*((0.5*wing.c*(1+wing.TR))^2); % uses average chord
Iyy_self(2,:) = (1/12)*W(2,:)*(6*(body.D/2)^2 + body.L^2);
Iyy_self(3,:) = (1/12)*W(3,:)*((0.5*h_tail.c*(1+h_tail.TR))^2);
Iyy_self(4,:) = (1/12)*W(4,:)*((v_tail.b/2)^2 + (0.5*v_tail.c*(1+v_tail.TR))^2);
Iyy_self(5,:) = 0;
Iyy_self(6,:) = (1/12)*W(6,:)*(3*(nacelle.D/2)^2 + nacelle.L^2);
Iyy_self(7,:) = 0;
Iyy_self(8,:) = 0;
Iyy_self(9,:) = 0;
Iyy_self(10,:) = (1/12)*W(10,:)*((0.5*wing.c*(1+wing.TR))^2);
Iyy_self(11,:) = (1/12)*W(11,:)*(3*(body.D/2)^2 + body.tank_length^2);

Iyy_all = Iyy_self + W.*(x.^2 + z.^2);
Iyy = sum(Iyy_all,1);

%% Izz
Izz_self = zeros(11,3);

Izz_self(1,:) = (1/12)*W(1,:)*(wing.b^2 + (0.5*wing.c*(1+wing.TR))^2); % uses average chord
Izz_self(2,:) = (1/12)*W(2,:)*(6*(body.D/2)^2 + body.L^2);
Izz_self(3,:) = (1/12)*W(3,:)*(h_tail.b^2 + (0.5*h_tail.c*(1+h_tail.TR))^2);
Izz_self(4,:) = (1/12)*W(4,:)*((0.5*v_tail.c*(1+v_tail.TR))^2);
Izz_self(5,:) = 0;
Izz_self(6,:) = (1/12)*W(6,:)*(3*(nacelle.D/2)^2 + nacelle.L^2);
Izz_self(7,:) = 0;
Izz_self(8,:) = 0;
Izz_self(9,:) = 0;
Izz_self(10,:) = (1/12)*W(10,:)*((wing.b/2)^2 + (0.5*wing.c*(1+wing.TR))^2);
Izz_self(11,:) = (1/12)*W(11,:)*(3*(body.D/2)^2 + body.tank_length^2);

Izz_all = Izz_self + W.*(x.^2);
Izz = sum(Izz_all,1); % Should output 1 x 3 array

%% TODO: add actual xz inertia
% Treat everything as a point mass
Ixz_all = W.*x.*z;
Ixz = sum(Ixz_all,1);

%% Store Values
plane.geo.Ixx = Ixx;
plane.geo.Iyy = Iyy;
plane.geo.Izz = Izz;
plane.geo.Ixz = Ixz;

%% store I's
% I = zeros(2,3);
% I(1,1) = Ixx_dry; I(2,1) = Ixx_wet;
% I(1,2) = Iyy_dry; I(2,2) = Iyy_wet;
% I(1,3) = Izz_dry; I(2,3) = Izz_wet;
% I = I/32.2;

%% Ixz


end

% %% Ixx
% %Ixx_wing = (1/12)*(W(1)*wing.b^2) + 0.25*(W(1)*wing.c^2);
% Ixx_wing = (1/12)*(W(1)*(wing.b/2)^2);
% Ixx_tail = (1/12)*(x(3)^2)*(W(3)+W(4));
% Ixx_fuel = (1/12)*(W(10)*wing.b^2) + W(10)*x(10)^2;
% Ixx_engines = (plane.prop.W*(wing.b/4)^2);
% Ixx_fuselage = (1/2)*(W(2)*(x(2)^2));
% Ixx_retardent = (2/5)*W(11)*body.W^2;
% 
% Ixx_dry = Ixx_wing + Ixx_tail  + Ixx_fuel + Ixx_engines + Ixx_fuselage;
% Ixx_wet = Ixx_dry + Ixx_retardent;
% 
% %% Iyy
% Iyy_wing = (1/2)*(W(1)*wing.c^2) + W(1)*(x(1)^2) ;
% Iyy_tail = (1/2)*(x(3)^2)*(W(3)+W(4)) + 0.25*((W(3)+W(4)))*(v_tail.b^2) + (W(3)+W(4))*(x(3)^2);
% Iyy_fuel = (1/12)*(W(10)*wing.b^2)+W(10)*(x(1)^2) ;
% Iyy_engines = (plane.prop.W*(wing.b/2)^2) +W(6)*(x(1)^2) ;
% Iyy_fuselage = (1/12)*(W(2)*body.L^2);
% Iyy_retardent = (2/5)*W(11)*body.W^2;
% 
% Iyy_dry = Iyy_wing + Iyy_tail  + Iyy_fuel + Iyy_engines + Iyy_fuselage;
% Iyy_wet = Iyy_dry + Iyy_retardent;
% 
% 
% %% Izz
% Izz_wing = (1/12)*(W(1)*wing.b^2) + W(1)*(x(1)^2);
% Izz_tail = (1/2)*(x(3)^2)*(W(3)+W(4)) + (W(3)+W(4))*(x(3)^2);
% Izz_fuel = (1/12)*(W(10)*wing.b^2)+W(10)*(x(1)^2) ;
% Izz_engines = (plane.prop.W*(wing.b/3)^2) +W(6)*(x(1)^2) ;
% Izz_fuselage =  + (1/12)*(W(2)*body.L^2);
% Izz_retardent = (2/5)*W(11)*body.W^2;
% 
% Izz_dry = Izz_wing + Izz_tail  + Izz_fuel + Izz_engines + Izz_fuselage;
% Izz_wet = Izz_dry + Izz_retardent;
% %% TODO: add actual xz inertia
% Ixz_wet = (Ixx_wet+Iyy_wet)/2;
% Ixz_dry = (Ixx_dry+Iyy_dry)/2;
% 
% plane.geo.Ixx = [Ixx_wet; Ixx_dry];
% plane.geo.Iyy = [Iyy_wet; Iyy_dry];
% plane.geo.Izz = [Izz_wet; Izz_dry];
% plane.geo.Ixz = [Ixz_wet; Ixz_dry];
% 
% %% store I's
% I = zeros(2,3);
% I(1,1) = Ixx_dry; I(2,1) = Ixx_wet;
% I(1,2) = Iyy_dry; I(2,2) = Iyy_wet;
% I(1,3) = Izz_dry; I(2,3) = Izz_wet;
% I = I/32.2;

%% Ixz