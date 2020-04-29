function [newRocket] = checkFlutter(newRocket,T,P)
% marker 1 = pass

% Shear Modulus of Carbon Fiber
G = 4.5e6;            % psi
T = 518.7;             % Rankine (SL)
T = T - 459.67;        % Convert to F
AR = newRocket.geo.fin.AR;
lamba = newRocket.geo.fin.TR;
tc = newRocket.geo.fin.ThR;
a = sqrt(1.4*1716.59*(T + 460));
P = 2116*((T+459.7)/518.6)^(5.256);

% Flutter Speed

A = 1.337*AR^3*P*(lamba + 1)/(2*(AR+2)*tc^3);

newRocket.aero.flutter = a*sqrt(G/A);
    
end

