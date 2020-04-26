function [rocket] = getWeightLength(rocket)
    Weight_guess = 1000;  %lbs, initial weight guess
    W_dry = zeros(1,20);
    W_dry(1) = Weight_guess;
    
    %% Fin Weight
    S = rocket.geo.wing.S;                           %wing area, ft^2   % 
    AR = rocket.geo.wing.AR;                         %aspect ratio
    
    N = 5;%rocket.data.N;                                % Ultimate Load Factor (1.5 times limit load factor)(GIVEN)
    sweep_angle = rocket.geo.wing.sweep * (3.14/180);    % Deg %Wing 1/4 chord sweep angle
    taper_ratio = rocket.geo.wing.TR;                    %Taper Ratio
    thickness_ratio_wing = rocket.geo.wing.ThR;          %Maximum Thickness Ratio (GIVEN)
    v_max = rocket.data.requirements.v_max*0.593;        %FIX UNITS         %kts   %Equivalent Vmax at SL
    %W_wing = 96.948 * ((Weight * N/10^5)^0.65*(AR/cos(sweep_angle))^0.57*(S/100)^0.61*((1 + taper_ratio)/(2*thickness_ratio_wing))^0.36*(1+v_max/500)^0.5)^0.993;
    W_fins = 14*rocket.geo.fin.n; % lbm, from aerobee
    
    %% Fuselage Weight
    % - estimation stuff from 154A below
    %L_body = rocket.geo.body.L;     %ft       %Fuselage Length
    %width_fuselage = rocket.geo.body.W;      %ft        %Fuselage Width
    %depth_fuselage = rocket.geo.body.D;  %ft          %Fuselage Max Depth    
    %Weight_fuselage = 200*((Weight*N/10^5)^0.286*(L_body/10)^0.857*((width_fuselage+depth_fuselage)/10)*(v_max/100)^0.338)^1.1;
    W_body = 50; % lbm, complete guess TODO refine
    %% Nosecone Weight/Length
    W_nosecone = 5;
    % Assume conic nosecone
    alpha = 15;
    L_nosecone = (rocket.geo.body.D/2)/tand(alpha);
    

    %% TOTAL STRUCTURAL WEIGHT
    %TODO: add a better estimate of structural support, carbon fibre,
    %stringers etc. mostly dependent on total length so this probably needs
    %to be moved to the end
    W_struct = W_fins + W_body + W_nosecone;

    %% Propulsion System Weight
    %% Propellant Tank Weights
    rho_al = 168.555; % aluminum 6061-T6 density, lbm/ft3
    % Oxygen Tank
    D = rocket.geo.body.D;
    t = rocket.prop.t_ox;
    L = rocket.prop.L_ox;
    L_oxtank = 12*L+D; % tank length, inches
    V1 = pi/4*(D^2-(D-2*t)^2)*L; % cylindrical section metal volume, in3
    V2 = 4*pi/3*(D^3-(D-2*t)^3); % spherical section metal volume, in3
    W_oxtank = (V1+V2)*rho_al/12^3; % lbm
    % Fuel tank
    t = rocket.prop.t_fuel;
    L = rocket.prop.L_fuel;
    L_fueltank = 12*L+D; % tank length, inches
    V1 = pi/4*(D^2-(D-2*t)^2)*L; % cylindrical section metal volume
    V2 = 4*pi/3*(D^3-(D-2*t)^3); % spherical section metal volume
    W_fueltank = (V1+V2)*rho_al/12^3; % lbm
    %% Propellant Weights
    W_ox = rocket.prop.m_ox;
    W_fuel = rocket.prop.m_fuel;
    
    %% Engine Weight/Length
    rho_steel = 490.752; % steel density
    t = rocket.prop.tc;
    OD = rocket.prop.OD;
    A = pi/4*(OD^2-(OD-2*t)^2);
    L_engine = rocket.prop.Lc + rocket.prop.Lfrustum + rocket.prop.Ln; % in
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

    %% TOTAL WEIGHT
    W_dry = W_struct + W_propulsion + W_payload;
    W_wet = W_dry + W_ox + W_fuel;

    % TODO: enumerate weights for CG calc
    Weight = zeros(13,1);
    Weight(1) = W_fins;
    Weight(2) = W_body;
    Weight(3) = W_payload;
    Weight(5) = W_dry;
    rocket.data.weight.W = Weight; %TODO
    
    rocket.data.weight.oxtank = W_oxtank;
    rocket.data.weight.fueltank = W_fueltank;
    rocket.data.weight.engine = W_engine;
    rocket.data.weight.propulsion = W_propulsion;
    rocket.data.weight.dry = W_dry;
    rocket.data.weight.wet = W_wet;
    
    rocket.data.length.L = L_nosecone+L_payload+L_oxtank+L_fueltank+L_engine; % total length, inches
    rocket.data.length.nosecone = L_nosecone;
    rocket.data.length.payload = L_payload;
    rocket.data.length.engine = L_engine;
    rocket.data.length.oxtank = L_oxtank;
    rocket.data.length.fueltank = L_fueltank;
    rocket.geo.LD = rocket.data.length.L/rocket.geo.body.D;
    
end
