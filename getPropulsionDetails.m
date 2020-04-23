function [rocket] = getPropulsionDetails(rocket)


%% Find optimal OF at given chamber pressure and expansion altitude

PC = rocket.prop.PC;

[~,P_e,~] = getAtm(rocket.prop.expansion_h,0); % exit pressure in psi
rocket.prop.P_e = P_e;
load('LOXCH4comb.mat','combustion');
of = 2.5:0.0001:2.75;
cstar = zeros(1,length(of));
ct = zeros(1,length(of));
Isp = zeros(1,length(of));
for j = 1:length(of)
    cstar(j) = combustion.cstar(PC,of(j));
    gam = combustion.gam(PC,of(j));
    c1 = gam+1;c2 = gam - 1;
    c3 = c1./c2;
    ct(j) = sqrt(2*gam^2/c2*(2/c1)^c3*(1-(P_e/PC)^(c2/gam)));
    Isp(j) = cstar(j)*ct(j)/9.81;
end
%%
% figure
% plot(of,cstar)
% hold on
% yyaxis right
% plot(of, Isp)

[Isp_max, maxInd] = max(Isp);
rocket.prop.Isp = Isp_max; % specific impulse at max thrust
rocket.prop.OF = of(maxInd); % mixture ratio
rocket.prop.cstar = cstar(maxInd); % characteristic velocity, m/s
rocket.prop.ct = ct(maxInd); % thrust coefficient, dim.
rocket.prop.T = combustion.T(PC, rocket.prop.OF); % adiabatic flame temperature, K
rocket.prop.gam = combustion.gam(PC, rocket.prop.OF); % specific heat ratio
rocket.prop.mw = combustion.mw(PC, rocket.prop.OF); % molecular weight

rocket.prop.mdot = rocket.prop.F/rocket.prop.Isp; % mass flow rate, lbm/s
rocket.prop.At = rocket.prop.mdot*rocket.prop.cstar*3.28/rocket.prop.PC/32.174; % throat area, in2
rocket.prop.Dt = sqrt(4*rocket.prop.At/pi); % throat diameter, in
rocket.prop.t_b = rocket.prop.m_p / rocket.prop.mdot; % burn time, seconds

%% Chamber sizing
rocket.prop.Lstar = 50; % characteristic chamber length, in, dependent on propellant
rocket.prop.FOS = 1.5; % chamber factor of safety

rocket.prop.Vc = rocket.prop.Lstar*rocket.prop.At; % in3, chamber volume
sigma_y_steel = 42100; % yield strength of 303 steel, psi
rocket.prop.tc = PC*rocket.prop.OD/2/(sigma_y_steel/rocket.prop.FOS); % in
rocket.prop.ID = rocket.prop.OD - 2*rocket.prop.tc;
rocket.prop.Ac = pi/4*rocket.prop.ID^2;

rocket.prop.cont = rocket.prop.Ac/rocket.prop.At; % contraction ratio
rocket.prop.Lfrustum = rocket.prop.ID/3; % conical frustum length, in
% chamber length
rocket.prop.Lc = (rocket.prop.Vc-rocket.prop.Lfrustum*rocket.prop.Ac*(1+sqrt(1/rocket.prop.cont)+1/rocket.prop.cont))/rocket.prop.Ac;
%^ from rocket propulsion elements
%% nozzle sizing
gam = rocket.prop.gam;c1=gam+1;c2=gam-1;
rocket.prop.Me = sqrt(2/c2*((PC/P_e)^(c2/gam)-1));
rocket.prop.Ae = rocket.prop.At/rocket.prop.Me*((1+c2/2*rocket.prop.Me^2)/(c1/2))^(c1/(2*c2));
rocket.prop.De = sqrt(4*rocket.prop.Ae/pi);
rocket.prop.exp = rocket.prop.Ae/rocket.prop.At;
% assume conicalnozzle, can refine later to be bell nozzle to minimize
% divergence losses
alpha = 15; % divergent half angle, degrees
% nozzle length, in
rocket.prop.Ln = ((rocket.prop.De-rocket.prop.Dt)/2)/tand(alpha);

% Tank sizing
rocket.prop.m_ox = rocket.prop.m_p*(rocket.prop.OF/(rocket.prop.OF+1));
rocket.prop.m_fuel = rocket.prop.m_p*(1/(rocket.prop.OF+1));
rocket.prop.mdot_ox = rocket.prop.m_ox/rocket.prop.t_b;
rocket.prop.mdot_fuel = rocket.prop.m_fuel/rocket.prop.t_b;

rocket.prop.DPox = rocket.prop.PC*1.2 + 50; % pressure drop from ox tank to chamber, psi
rocket.prop.DPfuel = rocket.prop.PC*1.2 + 100; % pressure drop from fuel tank to chamber, psi
rocket.prop.P_ox = rocket.prop.PC + rocket.prop.DPox;
rocket.prop.P_fuel = rocket.prop.PC + rocket.prop.DPfuel;

sigma_y_al = 40000; % yield strength of 6061-T6 aluminum, psi
rho_ox = 72.2; % oxygen density, lbm/ft3
rho_fuel = 27.3753; % methane density, lbm/ft3
rocket.prop.V_ox = rocket.prop.m_ox/rho_ox; % ox volume, ft3
rocket.prop.V_fuel = rocket.prop.m_fuel/rho_fuel; % fuel volume, ft3
rocket.prop.t_ox = rocket.prop.P_ox*rocket.geo.body.D/2/(sigma_y_al/rocket.prop.FOS); % ox tank thickness, in
rocket.prop.t_fuel = rocket.prop.P_fuel*rocket.geo.body.D/2/(sigma_y_al/rocket.prop.FOS); % ox tank thickness, in

Vcap = 4*pi/3*(rocket.geo.body.D/2/12)^3; % tank cap volume (for 2 caps)
rocket.prop.L_ox = rocket.geo.body.D/12 + (rocket.prop.V_ox-Vcap)/(pi/4*(rocket.geo.body.D/12)^2); % ox tank length, ft
rocket.prop.L_fuel = rocket.geo.body.D/12 + (rocket.prop.V_fuel-Vcap)/(pi/4*(rocket.geo.body.D/12)^2); % fuel tank length, ft


end
