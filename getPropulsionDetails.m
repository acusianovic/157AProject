function [rocket] = getPropulsionDetails(rocket)

[rocket.prop.OF, rocket.prop.T, rocket.prop.gamma] = getCombustion(rocket.prop.PC);

rocket.prop.cstar = ;


rocket.prop.At = rocket.prop.mdot*rocket.prop.cstar/rocket.prop.PC;

% chamber sizing
rocket.prop.Lstar = 50; % characteristic chamber length, in
rocket.prop.eps = 3; % contraction ratio
rocket.prop.FOS = 1.8; % chamber factor of safety

sigma_y_steel = 42100; % yield strength of 303 steel, psi

D = 1.25; % diameter, ft
rocket.prop.t = rocket.prop.PC*12*D/2/(sigma_y_steel/rocket.prop.FOS); % in




end
