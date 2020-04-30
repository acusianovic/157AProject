function [rocket] = getWeightLength(rocket)
    Weight_guess = 1000;  %lbs, initial weight guess
    W_dry = zeros(1,20);
    W_dry(1) = Weight_guess;
        
    %% Nosecone
    L_nosecone = rocket.geo.nc.L*12; % inches
    rho_nosecone = 100;        % Fiberglass (lb/ft^3), change later
    W_nosecone = rho_nosecone*rocket.geo.nc.V; % ft3
    
    %% Payload
    W_payload = 10;    % lbs
    L_payload = 12;    % inches, from requirements
    

    %% Recovery
    L_recovery = 7; % estimate, recovery bay length, inches
    W_recovery = 2; % estimate
    
    %% Propulsion System Weight 
    %% Plumbing Bay 1
    W_pbay1 = 5; % lbm
    L_pbay1 = 6; % inches
    
    %% Propellant Tanks
    rho_al = 168.555; % aluminum 6061-T6 density, lbm/ft3
    D = rocket.geo.body.D; % in
    
    %% Pressurant Tank
    t = rocket.prop.t_press; % in
    L_cyl = rocket.prop.L_press - D; % cyldinrical section, inches
    L_presstank = rocket.prop.L_press; % tank length, inches
    V1 = pi/4*(D^2-(D-2*t)^2)*L_cyl; % cylindrical section metal volume, in3
    V2 = 4*pi/3*((D/2)^3-((D-2*t)/2)^3); % spherical section metal volume, in3
    W_presstank = (V1+V2)*rho_al/12^3; % lbm
    W_press = rocket.prop.m_press;
    
    %% Oxygen Tank
    t = rocket.prop.t_ox; % in
    L_cyl = rocket.prop.L_ox - D; % in
    V1 = pi/4*(D^2-(D-2*t)^2)*L_cyl; % cylindrical section metal volume, in3
    V2 = 4*pi/3*((D/2)^3-((D-2*t)/2)^3); % spherical section metal volume, in3 
    W_oxtank = (V1+V2)*rho_al/12^3; % lbm
    W_ox = rocket.prop.m_ox;
    L_oxtank = rocket.prop.L_ox; % tank length, inches
    
    %% Fuel tank
    t = rocket.prop.t_fuel;% in
    L_cyl = rocket.prop.L_fuel - D; % cyldinrical section, inches
    L_fueltank = rocket.prop.L_fuel; % tank length, inches
    V1 = pi/4*(D^2-(D-2*t)^2)*L_cyl; % cylindrical section metal volume, in3
    V2 = 4*pi/3*((D/2)^3-((D-2*t)/2)^3); % spherical section metal volume, in3
    W_fueltank = (V1+V2)*rho_al/12^3; % lbm
    W_fuel = rocket.prop.m_fuel;
    
    %% Plumbing Bay 2
    W_pbay2 = 5; % lbm
    L_pbay2 = 6; % inches
    
    %% Engine
    rho_steel = 490.752; % steel density
    t = rocket.prop.tc;
    OD = rocket.prop.OD;
    A = pi/4*(OD^2-(OD-2*t)^2);
    L_engine = rocket.prop.Lc + rocket.prop.Lfrustum + rocket.prop.Ln; % in
    % approximate total weight assuming total length is cylinder
    W_engine = A*L_engine*rho_steel/12^3; % lbm

    %% Body
    L_body = L_payload+L_recovery+L_presstank+L_pbay1+L_oxtank+L_fueltank+L_pbay2+L_engine;
    V_body = (pi/4) * ((D+rocket.geo.nc.tn)^2 - D^2);
    %W_body = V_body * rho_al; % change density later
    W_body = 70;
    
    %% Fins
    S = rocket.geo.fin.S;                  %fin  area, ft^2   %
    rho_CF = 111.24;                       % density of carbon fiber, lb/ft3
    W_fin = S*rocket.geo.fin.ThR*rocket.geo.fin.c*0.5*rho_CF;  
    W_fins = W_fin*rocket.geo.fin.n; % lbm
    rocket.geo.fin.LE = L_body - rocket.geo.fin.c*12; % place fin at bottom of the rocket

    %% TOTAL WEIGHTS
    W_propulsion = W_presstank+W_pbay1+W_oxtank+W_fueltank+W_pbay2+W_engine;
    W_struct = W_fins + W_body + W_nosecone;
    W_dry = W_struct + W_propulsion + W_payload + W_recovery;
    W_wet = W_dry + W_ox + W_fuel + W_press;

    % For CG Calculation (Weight)
    rocket.data.weight.body = W_body;
    rocket.data.weight.nosecone = W_nosecone;
    rocket.data.weight.payload = W_payload;
    rocket.data.weight.recovery = W_recovery;
    rocket.data.weight.presstank = W_presstank;
    rocket.data.weight.press = W_press;
    rocket.data.weight.pbay1 = W_pbay1;
    rocket.data.weight.oxtank = W_oxtank;
    rocket.data.weight.ox = W_ox;
    rocket.data.weight.fuel = W_fuel;
    rocket.data.weight.fueltank = W_fueltank;
    rocket.data.weight.pbay2 = W_pbay2;
    rocket.data.weight.engine = W_engine;
    rocket.data.weight.fins = W_fins;
    
    rocket.data.weight.propulsion = W_propulsion;
    rocket.data.weight.dry = W_dry;
    rocket.data.weight.wet = W_wet;

    %% TOTAL LENGTHS
    % In order from top of rocket
    rocket.data.length.nosecone = L_nosecone;
    rocket.data.length.payload = L_payload;
    rocket.data.length.recovery = L_recovery;
    rocket.data.length.presstank = L_presstank;
    rocket.data.length.pbay1 = L_pbay1;
    rocket.data.length.fueltank = L_fueltank;
    rocket.data.length.oxtank = L_oxtank;
    rocket.data.length.pbay2 = L_pbay2;
    rocket.data.length.engine = L_engine;

    rocket.geo.body.L = L_body;    % Body lengthm inches
    rocket.data.length.body = L_body;    % Body lengthm inches
    rocket.data.length.L = L_nosecone + L_body; % total length, inches
    
    rocket.geo.LD = rocket.data.length.L/rocket.geo.body.D; % fineness ratio

    
end