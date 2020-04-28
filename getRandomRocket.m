function [rocket] = getRandomRocket(rocket)
%randomization format
%rand_value = min_value + rand(1)*(max_value - min_value)

%% Body
rocket.geo.body.D = 7 + rand()*(18 - 7); %in, body diameter
% rocket.geo.body.L = 10 + rand()*(20 - 10); %ft, fuselage L, define length
% later to be dependent on propellant volume

%% Nosecone
rocket.geo.nc.L = 2 + rand()*(5 - 2);
rocket.geo.nc.Shape = randi(3);
rocket.geo.nc.tn = 0.12;        % Thickness, in (change later)

% Calculate Volume
dx = 0.01;
x = 0:dx:rocket.geo.nc.L;
R = rocket.geo.body.D/2;

if rocket.geo.nc.Shape == 1         % Von Karman
    theta = acos( 1 - (2*x)/rocket.geo.nc.L);
    y = (R/sqrt(pi)) * sqrt(theta - sin(2.*theta)/2);
    yInner = y - rocket.geo.nc.tn/12;      % in ft
  % Zero out yInner
    for ii = 1:length(y)
        if yInner(ii) < 0
            yInner(ii) = 0;
        else
            break
        end
    end
  % Rotating curves around x axis
    VOuter = pi*trapz(x,y);
    VInner = pi*trapz(x,yInner);
    
elseif rocket.geo.nc.Shape == 2     % 1/2 Power
    y = R * (x/rocket.geo.nc.L).^(1/2);
    yInner = y - rocket.geo.nc.tn/12;      % in ft
  % Zero out yInner
    for ii = 1:length(y)
        if yInner(ii) < 0
            yInner(ii) = 0;
        else
            break
        end
    end
  % Rotating curves around x axis
    VOuter = pi*trapz(x,y);
    VInner = pi*trapz(x,yInner);
    
elseif rocket.geo.nc.Shape == 3     % Elliptical
    y = R * sqrt(1-(x/rocket.geo.nc.L).^2);
    yInner = y - rocket.geo.nc.tn/12;      % in ft
  % Zero out yInner
    for ii = 1:length(y)
        if yInner(ii) < 0
            yInner(ii) = 0;
        else
            break
        end
    end
  % Rotating curves around x axis
    VOuter = pi*trapz(x,y);
    VInner = pi*trapz(x,yInner);
end

rocket.geo.nc.V = VOuter - VInner; % Volume in ft^3;

%% fin, NACA 0010 Airfoil
rocket.geo.fin.n = 4; % number of fins (all values below for 1 fin)
rocket.geo.fin.TR = 0.5; % taper ratio
rocket.geo.fin.S = 5 + rand()*(30 - 5); %ft^2, fin area
rocket.geo.fin.AR = 2 + rand()*(4 - 2); %fin aspect ratio
rocket.geo.fin.b = (rocket.geo.fin.S * rocket.geo.fin.AR )^0.5; %ft, fin span length
rocket.geo.fin.c = 2*rocket.geo.fin.S/rocket.geo.fin.b/(1+rocket.geo.fin.TR); %ft, fin chord length
rocket.geo.fin.ThR = 0.1; % thickness ratio
rocket.geo.fin.sweep = 20 + rand()*(45-20); %degrees, sweep angle
rocket.geo.fin.S_wet = rocket.geo.fin.S*(1.977 + 0.52*rocket.geo.fin.ThR); %ft^2, wetted area formula from http://www.ipublishing.co.in/jarvol1no12010/EIJAER2011.pdf
rocket.geo.fin.h_t = 0.3; %nondimensional distance to maximum thickness

rocket.geo.fin.LE = rocket.geo.body.L - rocket.geo.fin.c; % place fin at bottom of the rocket

rocket.geo.fin.h_ac = 0.25; %nondimensional, distance from fin leading edge to AC
rocket.geo.fin.ac = rocket.geo.fin.h_ac*rocket.geo.fin.c; %ft, distance from fin leading edge to AC, set to quarter chord
rocket.geo.fin.cl_a = 6.8209; %Cl/rad for NACA 0010 airfoil
rocket.geo.fin.cl_0 = 0; % Cl for 0 AOA for NACA 0010 airfoil

%% Randomize Propultion Parameters
% rocket.prop.m_p = 600 + rand()*(1200-600); % propellant weight, lbm
% randomize burn time instead
rocket.prop.PC = 200 + rand()*(600-200); % chamber pressure, psi
rocket.prop.OD = rocket.geo.body.D*(0.6 +rand()*(0.9 - 0.6)); % rocket chamber OD, in
rocket.prop.Itot = 1E3*(170 + rand()*(200 - 180)); % total impulse, lbf-s
rocket.prop.F = 1000 + rand()*(5000-2000); % thrust, lbf


% rocket.prop.A_e = pi/4*rocket.geo.body.D^2; % exit area, max at rocket diameter
rocket.prop.expansion_h = 30000; % expansion altitude, maybe randomize later?
% or possibly use this as a starting point and then once we nail down a
% rocket we can float this to maximize altitude


end

% function [plane] = getRandomPlaneOld(plane)
% 
% %randomization format
% %rand_value = min_value + rand(1)*(max_value - min_value)
% V_tank = 256; %ft3 volume of retardent tank, sized to fit 16000 lbs of retardent
% %fuselage
% plane.geo.body.W = 2 + rand(1)*(6 - 2); %ft, fuselage width
% plane.geo.body.D = plane.geo.body.W; %ft, fuselage depth, for a circular cross section plane
% plane.geo.body.L = 50 + rand(1)*(120 - 50); %ft, fuselage L
% tank_length = [(V_tank - (8*3.1415/3)*(plane.geo.body.D/2)^3)/(3.1415*(plane.geo.body.D/2)^2)]+plane.geo.body.D;
%
% %% fin
% plane.geo.fin.cl_a = 6.88; %Cl/rad for NACA 6412 airfoil
% plane.geo.fin.cl_0 = 0.7626; %Cl for 0 AOA for NACA 6412 airfoil
% plane.geo.fin.TR = 0.1 + rand(1)*(1 - 0.1); % taper ratio
% plane.geo.fin.S = 500 + rand(1)*(1200 - 500); %ft^2, fin area
% plane.geo.fin.b = 60 + rand(1)*(150 - 60); %ft, finspan
% plane.geo.fin.AR = plane.geo.fin.b^2 / plane.geo.fin.S; %fin aspect ratio
% plane.geo.fin.c = 2*plane.geo.fin.S / (plane.geo.fin.b*(1+plane.geo.fin.TR)); %ft, fin chord length
% plane.geo.fin.ThR = 0.12; % thickness ratio
% plane.geo.fin.sweep = 0 + rand(1)*(5-0); %degrees, sweep length
% plane.geo.fin.S_wet = plane.geo.fin.S*(1.977 + 0.52*plane.geo.fin.ThR); %ft^2, wetted area formula from http://www.ipublishing.co.in/jarvol1no12010/EIJAER2011.pdf
% plane.geo.fin.h_t = 0.301; %nondimensional distance to maximum thickness
%
% plane.geo.fin.LE = (0.1*plane.geo.body.L) + rand(1)*(plane.geo.body.L*(0.9-0.1));
% % need to define distance from nosetip to fin edge
% plane.geo.fin.h_ac = 0.25; %nondimensional, distance from fin leading edge to AC
% plane.geo.fin.ac = plane.geo.fin.h_ac*plane.geo.fin.c; %ft, distance from fin leading edge to AC, set to quarter chord
%
% %% horizontal tail
% %%%%%% We're using the same airfoil for the tail? %%%%%%%%%%%
% plane.geo.h_tail.cl_a = 6.88; %Cl/rad for NACA 6412 airfoil
% plane.geo.h_tail.cl_0 = 0.7626; %Cl for 0 AOA for NACA 6412 airfoil
% plane.geo.h_tail.TR = 0.1 + rand(1)*(1 - 0.1);
% Sht_Sw = 0.1 + rand(1)*(0.75 - 0.1);
% plane.geo.h_tail.S = Sht_Sw*plane.geo.fin.S; %ft^2, h_tail area
% plane.geo.h_tail.AR = 4 + rand(1)*(8 - 4); %h_tail aspect ratio
% plane.geo.h_tail.b = (plane.geo.h_tail.S * plane.geo.h_tail.AR)^0.5; %ft, h_tail span length
% plane.geo.h_tail.c = 2*plane.geo.h_tail.S / (plane.geo.h_tail.b*(1+plane.geo.h_tail.TR)); %ft, h_tail chord length
% plane.geo.h_tail.ThR = 0.12;
% plane.geo.h_tail.sweep = 0 + rand(1)*(5-0); %degrees, sweep length
% plane.geo.h_tail.S_wet = plane.geo.h_tail.S*(1.977 + 0.52*plane.geo.h_tail.ThR); %ft^2, wetted area formula from http://www.ipublishing.co.in/jarvol1no12010/EIJAER2011.pdf
% plane.geo.h_tail.h_t = 0.301; %nondimensional distance to maximum thickness
%
% fin = plane.geo.fin;
% h_tail_minLE = fin.LE + fin.c; % min location is at least end of fin
% h_tail_maxLE = plane.geo.body.L - plane.geo.h_tail.c; % max location has end of tail = end of body
% plane.geo.h_tail.LE = h_tail_minLE + rand(1)*(h_tail_maxLE - h_tail_minLE); %Dist from nose to LE of htail
%
% plane.geo.h_tail.ac = (plane.geo.h_tail.LE - plane.geo.fin.LE) + 0.25*plane.geo.h_tail.c/plane.geo.fin.c; %ft, distance from fin leading edge to htail AC, set to quarter chord
% plane.geo.h_tail.h_ac = plane.geo.h_tail.ac/plane.geo.fin.c; %nondimensional, distance from fin leading edge to htail AC
%
%
% %% vertical tail
% plane.geo.v_tail.ThR = 0.12;
% plane.geo.v_tail.TR =  0.1 + rand(1)*(1 - 0.1);
% Svt_Sw = 0.05 + rand(1)*(0.75 - 0.05);
% plane.geo.v_tail.S = Svt_Sw * plane.geo.fin.S; %ft^2, v_tail area
% plane.geo.v_tail.AR = 2 + rand(1)*(7 - 2); %v_tail aspect ratio
% plane.geo.v_tail.b = ((plane.geo.v_tail.S*plane.geo.v_tail.AR)^0.5)/2; %ft, v_tail span length
% plane.geo.v_tail.c = 4*plane.geo.v_tail.S / (plane.geo.v_tail.b*(1+plane.geo.v_tail.TR)); %ft, v_tail chord length
% plane.geo.v_tail.sweep = 0 + rand(1)*(15-0); %degrees, sweep length
% plane.geo.v_tail.S_wet = plane.geo.v_tail.S*(1.977 + 0.52*plane.geo.v_tail.ThR); %ft^2, wetted area formula from http://www.ipublishing.co.in/jarvol1no12010/EIJAER2011.pdf
% plane.geo.v_tail.h_t = 0.301; %nondimensional distance to maximum thickness
%
% v_tail_minLE = fin.LE + fin.c; % min location is end of fin
% v_tail_maxLE = plane.geo.body.L - plane.geo.v_tail.c; % max location has end of tail = end of body
% plane.geo.v_tail.LE = v_tail_minLE + rand(1)*(v_tail_maxLE - v_tail_minLE); %Dist from nose to LE of vtail
%
% plane.geo.v_tail.ac = (plane.geo.v_tail.LE - plane.geo.fin.LE) + 0.25*plane.geo.v_tail.c/plane.geo.fin.c; %ft, distance from fin leading edge to vtail AC, set to quarter chord
% plane.geo.v_tail.h_ac = plane.geo.v_tail.ac/plane.geo.fin.c; %nondimensional, distance from fin leading edge to vtail AC
% plane.geo.v_tail.cl_a = 0.1*180/3.1415; %vertical tail lift curve slope
%
% %% nacelle
% plane.geo.nacelle.L = 6; % Length of each nacelle (engine)
% plane.geo.nacelle.D = 1.585; % Diameter of nacelle (engine)
% plane.geo.nacelle.S_wet = 3.4*plane.geo.nacelle.L*plane.geo.nacelle.D;
%
%
% plane.prop.fuel_mass = 1125.9 + randn(1)*1125.9/30; %lb - guess based off of bessieMk3
%
% end
%
%
%

