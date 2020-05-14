function [rocket] = checkFlutter(rocket)

% Shear Modulus of Carbon Fiber
G = 4.5e6;            % psi
T = 518.7;             % Rankine (SL)
T = T - 459.67;        % Convert to F
AR = rocket.geo.fin.AR;
lamba = rocket.geo.fin.TR;
tc = rocket.geo.fin.ThR;
a = sqrt(1.4*1716.59*(T + 460));   % ft/s
P = 2116*((T+459.7)/518.6)^(5.256); % psf

% Flutter Speed

A = 1.337*AR^3*P*(lamba + 1)/(2*(AR+2)*tc^3);

rocket.data.aero.flutter = a*sqrt(G*144/A); % ft/s

end

