function [newrocket] =getCG(newrocket)

% Assume thin wall, so CG is not affected by thickness
% 1-D approximation

L_nose = newrocket.geo.nc.L;
x_nose = 0.424*L_nose;  % Assume to be elliptical

%% Body
%D = newrocket.geo.body.D;
L_body = newrocket.geo.length.body;
x_body = L_nose + (L_body/2);   % from nose

%% Payload
% Payload
W_payload = 10;     % lb
L_payload = newrocket.data.length.payload;
x_payload = L_nose + (L_payload/2);

%% Tanks
% OX tank
L_Ox = newrocket.data.length.oxtank;
x_Ox = L_nose + L_payload + (L_Ox/2);

% Fuel tank
L_fuel = newrocket.data.length.fueltank;
x_fuel = (L_nose + L_payload + L_Ox) + (L_fuel/2);

%% Engine
L_engine = newrocket.data.length.engine;
x_engine = (L_nose + L_payload + L_Ox + L_fuel) + (L_engine/2);

%% Fins
h = newrocket.geo.fin.b;
b = new.rocket.geo.fin.c;
a = newrocket.geo.fin.TR * b;
x_fins = (h/3) * (2*a + b)/(a + b);
x_fins = x_fins + (newrocket.data.length.L - b);

%% Force Balance

%Ignore line and valves weight for CG balancing

xCG_dry = ( (newrocket.data.weight.nosecone*x_nose) ...
    + (W_payload*x_payload) + (newrocket.data.weight.oxtank*x_Ox) ...
    - (newrocket.data.weight.body*x_body) ...
    - (newrocket.data.weight.fueltank*x_fuel) ...
    - (newrocket.data.weight.engine*x_engine) ... 
    - (newrocket.data.weight.fins*x_fins) )/newrocket.data.weight.dry;

xCG_wet = ( (newrocket.data.weight.nosecone*x_nose) ...
    + (W_payload*x_payload) + (newrocket.data.weight.oxtank*x_Ox) ...
    + (newrocket.data.weight.Ox*x_Ox) ...
    - (newrocket.data.weight.body*x_body) ...
    - (newrocket.data.weight.Fuel*x_fuel) ...
    - (newrocket.data.weight.fueltank*x_fuel) ...
    - (newrocket.data.weight.engine*x_engine) ... 
    - (newrocket.data.weight.fins*x_fins) )/newrocket.data.weight.wet;

newrocket.data.CG.dry = xCG_dry;
newrocket.data.CG.wet = xCG_wet;

end

