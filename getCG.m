function [rocket] = getCG(rocket)
% Assume thin wall, so CG is not affected by thickness
% 1-D approximation

L_nose = rocket.geo.nc.L*12; % in
x_nose = 0.424*L_nose;  % Assume to be elliptical

%% Body
% D = rocket.geo.body.D;
L_body = rocket.geo.length.body;
x_body = L_nose + (L_body/2);   % from nose


%% Payload
% Payload
W_payload = 10;     % lb
L_payload = rocket.data.length.payload;
x_payload = L_nose + (L_payload/2);

%% Recovery
L_recovery = rocket.data.length.recovery;
x_recovery = L_nose + L_payload + L_recovery/2;

%% Press tank
L_presstank = rocket.data.length.presstank;
x_presstank = L_nose + L_payload + L_recovery + L_presstank/2;

%% Pbay1
L_pbay1 = rocket.data.length.pbay1;
x_pbay1 = L_nose + L_payload + L_recovery + L_presstank + L_pbay1/2;

%% Tanks
% OX tank
L_ox = rocket.data.length.oxtank;
x_ox = L_nose + L_payload + L_recovery + L_presstank + L_pbay1 + (L_ox/2);

%% Fuel tank
L_fuel = rocket.data.length.fueltank;
x_fuel = L_nose + L_payload + L_recovery + L_presstank + L_pbay1 + L_ox + (L_fuel/2);

%% Pbay2
L_pbay2 = rocket.data.length.pbay2;
x_pbay2 = L_nose + L_payload + L_recovery + L_presstank + L_pbay1 + L_ox + L_fuel + L_pbay2/2;

%% Engine
L_engine = rocket.data.length.engine;
x_engine = L_nose + L_payload + L_recovery + L_presstank + L_pbay1 + L_ox + L_fuel + L_pbay2 + (L_engine/2);

%% Fins
h = rocket.geo.fin.b*12;
b = rocket.geo.fin.c*12;
a = rocket.geo.fin.TR * b;
x_fins = (h/3) * (2*a + b)/(a + b);
x_fins = (rocket.data.length.L - b) + x_fins;

%% Force Balance

%Ignore line and valves weight for CG balancing

xCG_dry = ( (rocket.data.weight.nosecone*x_nose) ...
    + (W_payload*x_payload) ...
    + (rocket.data.weight.recovery*x_recovery)...
    + (rocket.data.weight.presstank*x_presstank) ...
    + (rocket.data.weight.pbay1*x_pbay1)...
    + (rocket.data.weight.oxtank*x_ox) ...
    + (rocket.data.weight.body*x_body) ...
    + (rocket.data.weight.fueltank*x_fuel) ...
    + (rocket.data.weight.pbay2*x_pbay2) ...
    + (rocket.data.weight.engine*x_engine) ... 
    + (rocket.data.weight.fins*x_fins) )/rocket.data.weight.dry;

xCG_wet = ( (rocket.data.weight.nosecone*x_nose) ...
    + (W_payload*x_payload) ...
    + (rocket.data.weight.recovery*x_recovery)...
    + (rocket.data.weight.presstank*x_presstank) ...
    + (rocket.data.weight.press*x_presstank)...
    + (rocket.data.weight.pbay1*x_pbay1)...
    + (rocket.data.weight.oxtank*x_ox) ...
    + (rocket.data.weight.ox*x_ox) ...
    + (rocket.data.weight.body*x_body) ...
    + (rocket.data.weight.fueltank*x_fuel) ...
    + (rocket.data.weight.fuel*x_fuel) ...
    + (rocket.data.weight.pbay2*x_pbay2) ...
    + (rocket.data.weight.engine*x_engine) ... 
    + (rocket.data.weight.fins*x_fins) )/rocket.data.weight.wet;

rocket.data.CG.dry = xCG_dry;
rocket.data.CG.wet = xCG_wet;

end

