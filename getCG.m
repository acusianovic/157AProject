function [rocket] = getCG(rocket)

% Assume thin wall, so CG is not affected by thickness
% 1-D approximation

L_nose = rocket.geo.nc.L*12; % in
x_nose = 0.424*L_nose;  % Assume to be elliptical

%% Body
%D = rocket.geo.body.D;
L_body = rocket.geo.length.body;
x_body = L_nose + (L_body/2);   % from nose

%% Payload
% Payload
W_payload = 10;     % lb
L_payload = rocket.data.length.payload;
x_payload = L_nose + (L_payload/2);

%% Tanks
% OX tank
L_Ox = rocket.data.length.oxtank;
x_Ox = L_nose + L_payload + (L_Ox/2);

% Fuel tank
L_fuel = rocket.data.length.fueltank;
x_fuel = (L_nose + L_payload + L_Ox) + (L_fuel/2);

%% Engine
L_engine = rocket.data.length.engine;
x_engine = (L_nose + L_payload + L_Ox + L_fuel) + (L_engine/2);

%% Fins
h = rocket.geo.fin.b*12;
b = rocket.geo.fin.c*12;
a = rocket.geo.fin.TR * b;
x_fins = (h/3) * (2*a + b)/(a + b);
x_fins = x_fins + (rocket.data.length.L - b);

%% Force Balance

%Ignore line and valves weight for CG balancing

xCG_dry = ( (rocket.data.weight.nosecone*x_nose) ...
    + (W_payload*x_payload) + (rocket.data.weight.oxtank*x_Ox) ...
    + (rocket.data.weight.body*x_body) ...
    + (rocket.data.weight.fueltank*x_fuel) ...
    + (rocket.data.weight.engine*x_engine) ... 
    + (rocket.data.weight.fins*x_fins) )/rocket.data.weight.dry;

xCG_wet = ( (rocket.data.weight.nosecone*x_nose) ...
    + (W_payload*x_payload) + (rocket.data.weight.oxtank*x_Ox) ...
    + (rocket.data.weight.Ox*x_Ox) ...
    + (rocket.data.weight.body*x_body) ...
    + (rocket.data.weight.Fuel*x_fuel) ...
    + (rocket.data.weight.fueltank*x_fuel) ...
    + (rocket.data.weight.engine*x_engine) ... 
    + (rocket.data.weight.fins*x_fins) )/rocket.data.weight.wet;

rocket.data.CG.dry = xCG_dry;
rocket.data.CG.wet = xCG_wet;

end

