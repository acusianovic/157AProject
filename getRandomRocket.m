%% 
function [rocket] = getRandomRocket(rocket)
%randomization format
%rand_value = min_value + rand(1)*(max_value - min_value)

%% Body
rocket.geo.body.D = 9 + rand()*(13 - 9); %in, body diameter
% rocket.geo.body.L = 10 + rand()*(20 - 10); %ft, fuselage L, define length

%% Nosecone
rocket.geo.nc.L = 6 + rand()*(4 - 2); % feet
rocket.geo.nc.Shape = randi(3);
rocket.geo.nc.tn = 0.12;        % Thickness, in (change later)

% Calculate Volume
dx = 0.01;
x = 0:dx:rocket.geo.nc.L;
R = rocket.geo.body.D/(12*2);

if rocket.geo.nc.Shape == 1         % Von Karman
    theta = acos( 1 - (2*x)/rocket.geo.nc.L);
    y = (R/sqrt(pi)) * sqrt(theta - sin(2.*theta)/2);
    yInner = y - rocket.geo.nc.tn/12;      % in ft
  % Zero out yInner
    for ii = 1:length(y)
        if yInner(ii) < 0
            yInner(ii) = 0;
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
        end
    end
  % Rotating curves around x axis
    VOuter = pi*trapz(x,y);
    VInner = pi*trapz(x,yInner);
end

% Derivative of yN
dy = zeros(1,length(y));
dy(1) = (y(2) - y(1))/dx;
dy(length(y)) = (y(length(y)) - y(length(y)-1))/dx;
for ii = 2:length(y)-1
    dy(ii) = ( y(ii+1) - y(ii) )/ dx;
end
% Surface Area of Nose Cone(ft^2)
rocket.geo.nc.S = 2*pi*trapz(x,y.*sqrt(1 + dy.^2));
rocket.geo.nc.V = VOuter - VInner; % Volume in ft^3;

%% fin, similiar shape to the Aerobee
rocket.geo.fin.n = 4; % number of fins (all values below for 1 fin)
rocket.geo.fin.TR = 0.5 + rand()*(1 - 0.5); % taper ratio
rocket.geo.fin.S = 2 + rand()*(5 - 2); % ft^2, fin area
rocket.geo.fin.AR = 0.1 + rand()*(1 - 0.1); %fin aspect ratio
rocket.geo.fin.b = (rocket.geo.fin.S * rocket.geo.fin.AR )^0.5; %ft, fin span length
rocket.geo.fin.c = 2*rocket.geo.fin.S/rocket.geo.fin.b/(1+rocket.geo.fin.TR); % ft, fin chord length
rocket.geo.fin.ThR = 1.1/(12*rocket.geo.fin.c); % thickness ratio
rocket.geo.fin.sweep = 20 + rand()*(45-20); %degrees, sweep angle
rocket.geo.fin.S_wet = rocket.geo.fin.S*(1.977 + 0.52*rocket.geo.fin.ThR); %ft^2, wetted area formula from http://www.ipublishing.co.in/jarvol1no12010/EIJAER2011.pdf
rocket.geo.fin.h_t = 0.3; %nondimensional distance to maximum thickness

rocket.geo.fin.h_ac = 0.25; %nondimensional, distance from fin leading edge to AC
rocket.geo.fin.ac = rocket.geo.fin.h_ac*rocket.geo.fin.c; %ft, distance from fin leading edge to AC, set to quarter chord
rocket.geo.fin.cl_a = 6.8209; %Cl/rad for NACA 0010 airfoil
rocket.geo.fin.cl_0 = 0; % Cl for 0 AOA for NACA 0010 airfoil

%% Randomize Propultion Parameters
% rocket.prop.m_p = 600 + rand()*(1200-600); % propellant weight, lbm
% randomize burn time instead
rocket.prop.PC = 250 + rand()*(400-250); % chamber pressure, psi
% rocket.prop.OD = rocket.geo.body.D*(0.7 +rand()*(0.9 - 0.7)); % determine
% from throat area instead
rocket.prop.Itot = 1E3*(80 + rand()*(130 - 80)); % total impulse, lbf-s
rocket.prop.F = 900 + rand()*(4000-900); % thrust, lbf


% rocket.prop.A_e = pi/4*rocket.geo.body.D^2; % exit area, max at rocket diameter
rocket.prop.expansion_h = 30000; % expansion altitude, maybe randomize later?
% or possibly use this as a starting point and then once we nail down a
% rocket we can float this to maximize altitude


end
