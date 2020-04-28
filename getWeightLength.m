function [newrocket] = getWeightLength(newrocket)
    Weight_guess = 1000;  %lbs, initial weight guess
    W_dry = zeros(1,20);
    W_dry(1) = Weight_guess;
        
    %% Nosecone Weight
    L_nosecone = newrocket.geo.nc.L;
    rho_nosecone = 100;        % Fiberglass (lb/ft^3), change later
    W_nosecone = rho_nosecone*newrocket.geo.nc.V;


    %% Propulsion System Weight
    %% Propellant Tank Weights
    rho_al = 168.555; % aluminum 6061-T6 density, lbm/ft3
    % Oxygen Tank
    D = newrocket.geo.body.D;
    t = newrocket.prop.t_ox;
    L = newrocket.prop.L_ox;
    L_oxtank = 12*L+D; % tank length, inches
    V1 = pi/4*(D^2-(D-2*t)^2)*L; % cylindrical section metal volume, in3
    V2 = 4*pi/3*(D^3-(D-2*t)^3); % spherical section metal volume, in3
    W_oxtank = (V1+V2)*rho_al/12^3; % lbm
    % Fuel tank
    t = newrocket.prop.t_fuel;
    L = newrocket.prop.L_fuel;
    L_fueltank = 12*L+D; % tank length, inches
    V1 = pi/4*(D^2-(D-2*t)^2)*L; % cylindrical section metal volume
    V2 = 4*pi/3*(D^3-(D-2*t)^3); % spherical section metal volume
    W_fueltank = (V1+V2)*rho_al/12^3; % lbm
    %% Propellant Weights
    W_ox = newrocket.prop.m_ox;
    W_fuel = newrocket.prop.m_fuel;
    
    %% Engine Weight/Length
    rho_steel = 490.752; % steel density
    t = newrocket.prop.tc;
    OD = newrocket.prop.OD;
    A = pi/4*(OD^2-(OD-2*t)^2);
    L_engine = newrocket.prop.Lc + newrocket.prop.Lfrustum + newrocket.prop.Ln; % in
    % approximate total weight assuming total length is cylinder
    W_engine = A*L_engine*rho_steel/12^3; % lbm
    
    %% Plumbing Weights
    W_valves = 10;
    W_lines = 5;
    
    %% TOTAL PROPULSION SYSTEM DRY WEIGHT 
    W_propulsion = W_oxtank+W_fueltank+W_engine+W_valves+W_lines;
    
    %% Payload Weight/Length
    W_payload = 10;    % lbs
    L_payload = 12;    % inches, from requirements
    
    %% Fuselage Weight
    L_body = L_payload+L_oxtank+L_fueltank+L_engine;
    V_body = (pi/4) * ((D+rocket.geo.nc.tn)^2 - D^2);
    W_body = V_body * rho_al; % change density later
    
    %% Fin Weight
    S = newrocket.geo.wing.S;                  %wing area, ft^2   % 
    AR = newrocket.geo.wing.AR;                %aspect ratio
    
    N = 5;%rocket.data.N;                           % Ultimate Load Factor (1.5 times limit load factor)(GIVEN)
    sweep_angle = newrocket.geo.wing.sweep ;        % Deg %Wing 1/4 chord sweep angle
    taper_ratio = newrocket.geo.wing.TR;            %Taper Ratio
    thickness_ratio_wing = newrocket.geo.wing.ThR;          %Maximum Thickness Ratio (GIVEN)
    v_max = newrocket.data.requirements.v_max*0.593;        %FIX UNITS         %kts   %Equivalent Vmax at SL
    %W_wing = 96.948 * ((Weight * N/10^5)^0.65*(AR/cos(sweep_angle))^0.57*(S/100)^0.61*((1 + taper_ratio)/(2*thickness_ratio_wing))^0.36*(1+v_max/500)^0.5)^0.993;
    W_fins = 14*newrocket.geo.fin.n; % lbm, from aerobee
   
    
    %% TOTAL STRUCTURAL WEIGHT
    %TODO: add a better estimate of structural support, carbon fibre,
    %stringers etc. mostly dependent on total length so this probably needs
    %to be moved to the end
    W_struct = W_fins + W_body + W_nosecone;

    %% TOTAL WEIGHT
    W_dry = W_struct + W_propulsion + W_payload;
    W_wet = W_dry + W_ox + W_fuel;

    % Check getCG function
%     % TODO: enumerate weights for CG calc
%     Weight = zeros(13,1);
%     Weight(1) = W_nosecone;
%     Weight(2) = W_payload;
%     Weight(3) = W_oxtank;
%     Weight(5) = W_dry;

    % For CG Calculation (Weight)
    newrocket.data.weight.body = W_body;
    newrocket.data.weight.nosecone = W_nosecone;
    newrocket.data.weight.fins = W_fins;
    newrocket.data.weight.oxtank = W_oxtank;
    newrocket.data.weight.Ox = W_ox;
    newrocket.data.weight.Fuel = W_fuel;
    newrocket.data.weight.fueltank = W_fueltank;
    newrocket.data.weight.engine = W_engine;
    newrocket.data.weight.propulsion = W_propulsion;
    newrocket.data.weight.dry = W_dry;
    newrocket.data.weight.wet = W_wet;
    
    newrocket.data.length.L = L_nosecone+L_payload+L_oxtank+L_fueltank+L_engine; % total length, inches
    newrocket.data.length.nosecone = L_nosecone;
    newrocket.data.length.payload = L_payload;
    newrocket.data.length.engine = L_engine;
    newrocket.data.length.oxtank = L_oxtank;
    newrocket.data.length.fueltank = L_fueltank;
    newrocket.geo.length.body = L_body;    % Body length
    newrocket.geo.LD = newrocket.data.length.L/newrocket.geo.body.D;

    
end