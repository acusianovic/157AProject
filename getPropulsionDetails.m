function [rocket] = getPropulsionDetails(rocket,atmo_dat)

%% Find optimal OF at given chamber pressure and expansion altitude

P_exit = lininterp1(atmo_dat.Z,atmo_dat.P,rocket.prop.expansion_h); % psi
%[~, ~, P_exit, ~] = atmos(rocket.prop.expansion_h/3.28);
%P_exit = P_exit/101325*14.7; % psi
%[~,P_exit,~] = getAtm(rocket.prop.expansion_h,0); % exit pressure in psi

rocket.prop.P_e = P_exit;
P_chamber = rocket.prop.PC;
load('LOXCH4comb.mat','combustion');
OF = 2.5:0.0001:3;

rocket.prop.cstar_eff = 0.9; % combustion efficiency
rocket.prop.ct_eff = 0.95; % nozzle efficiency

cstar = zeros(1,length(OF));
ct = zeros(1,length(OF));
Isp = zeros(1,length(OF));
gam = zeros(1,length(OF));
for j = 1:length(OF)
    cstar(j) = combustion.cstar(P_chamber,OF(j));
    gam(j) = combustion.gam(P_chamber,OF(j));
    c1 = gam(j)+1;c2 = gam(j)-1;
    c3 = c1./c2;
    ct(j) = sqrt(2*gam(j)^2/c2*(2/c1)^c3*(1-(P_exit/P_chamber)^(c2/gam(j))));
    Isp(j) = cstar(j)*ct(j)/9.81*rocket.prop.cstar_eff*rocket.prop.ct_eff;
end

cstar = cstar*rocket.prop.cstar_eff; % m/s
ct = ct*rocket.prop.ct_eff; % dim.

rocket.prop.t_b = rocket.prop.Itot/rocket.prop.F; % s
m_p = rocket.prop.Itot./Isp; % lbm
mdot = m_p/rocket.prop.t_b; % lbm/s
At = rocket.prop.Itot./(P_chamber.*ct.*rocket.prop.t_b); % in2

rocket.prop.Lstar = 50; % characteristic chamber length, in, dependent on propellant
rocket.prop.FOS = 1.5; % chamber factor of safety
Al6061 = struct('strength',40000,'density',168.6); % psia, lbm/ft3
SS310 = struct('strength',19300,'density',492.6);  % psia, lbm/ft3
COPV = struct('strength',87000,'density',124.855921);
%% Chamber sizing

V_chamber = rocket.prop.Lstar.*At; % in3
A_chamber = 5.*At; % in2
ID = sqrt(4*A_chamber/pi); % in
t_chamber = P_chamber.*ID/2/(SS310.strength/rocket.prop.FOS); % in
OD = ID + 2.*t_chamber * 2; % in, to account for double jacket;
theta = 30; % contraction angle degrees
Dt = sqrt(4.*At/pi); % throat diameter, in

L1 = (ID-Dt)/(2*tand(theta)); % conical frustum length, in
cont = A_chamber./At; % contraction ratio
L_chamber = (V_chamber-A_chamber.*L1)./(A_chamber.*(1+sqrt(At./A_chamber)+At./A_chamber));


%% nozzle sizing
c1=gam(j)+1;c2=gam(j)-1;
Me = sqrt(2./c2.*((P_chamber./P_exit).^(c2./gam)-1));
Ae = At./Me.*((1+c2./2.*Me.^2)./(c1./2)).^(c1./(2.*c2));
De = sqrt(4.*Ae./pi);
eps = Ae./At;
% assume conicalnozzle, can refine later to be bell nozzle to minimize
% divergence losses
alpha = 15; % divergent half angle, degrees
% nozzle length, in
Ln = ((De-Dt)/2)/tand(alpha);

L_engine = L_chamber+Ln+L1;
m_engine = ID+2.*t_chamber*pi.*L_engine*SS310.density/12^3;

%% Propellant Tank Sizing

m_ox = m_p.*(OF./(OF+1));
m_fuel = m_p.*(1./(OF+1));
mdot_ox = m_ox/rocket.prop.t_b;
mdot_fuel = m_fuel/rocket.prop.t_b;

DPox = P_chamber*0.2 + 50; % pressure drop from ox tank to chamber, psi
DPfuel = P_chamber*0.2 + 100; % pressure drop from fuel tank to chamber, psi
rocket.prop.P_ox = P_chamber + DPox;
rocket.prop.P_fuel = P_chamber + DPfuel;

rho_ox = 71.2; % oxygen density, lbm/ft3
rho_fuel = 26.4; % methane density, lbm/ft3

ullage_margin = 0.15;
V_ox = m_ox/rho_ox; % ox volume, ft3
V_fuel = m_fuel/rho_fuel; % fuel volume, ft3
V_oxtank = V_ox*(1+ullage_margin);
V_fueltank = V_fuel*(1+ullage_margin);

rocket.prop.P_press = 4500; % pressurant tank pressure, psi (max pressure of an airsoft tank)

V_press = (V_ox*rocket.prop.P_ox + V_fuel*rocket.prop.P_fuel)/rocket.prop.P_press; % total required pressurant volume at press tank pressure, ft3
VtotSTP = V_press*rocket.prop.P_press/14.7/12^3; % vol at STP, ft3 or SCF 
R_HE = 2.682897569; % psi*ft3/lb*R, for helium
m_press = rocket.prop.P_press.*V_press/(486*R_HE); % mass of helium, lbm

D = rocket.geo.body.D; % in
[rocket.prop.t_oxtank,L_oxtank,m_oxtank] = vessel(rocket.prop.P_ox,D,V_oxtank,Al6061,1.5); % in, in, lbm
[rocket.prop.t_fueltank,L_fueltank,m_fueltank] = vessel(rocket.prop.P_fuel,D,V_fueltank,Al6061,1.5); % in, in, lbm
[rocket.prop.t_presstank,L_presstank,m_presstank] = vessel(rocket.prop.P_press,D/2,V_press,Al6061,1.5); % in, in, lbm
%% Lowest system mass

mtot = m_ox + m_oxtank + m_fuel + m_fueltank + m_press + m_presstank + m_engine; % lbm
[~,Ind] = min(mtot);

rocket.prop.OF = OF(Ind); % mixture ratio, dim.
rocket.prop.Isp = Isp(Ind); % specific impulse, s
rocket.prop.cstar = cstar(Ind); % characteristic velocity, m/s
rocket.prop.ct = ct(Ind); % thrust coefficient, dim.
rocket.prop.T = combustion.T(P_chamber, rocket.prop.OF); % adiabatic flame temperature, K
rocket.prop.gam = combustion.gam(P_chamber, rocket.prop.OF); % specific heat ratio
rocket.prop.mw = combustion.mw(P_chamber, rocket.prop.OF); % molecular weight

rocket.prop.m_p = m_p(Ind);
rocket.prop.mdot = mdot(Ind);
rocket.prop.At = At(Ind);
rocket.prop.Dt = Dt(Ind);

rocket.prop.Lc = L_chamber(Ind);
rocket.prop.L_engine = L_engine(Ind);
rocket.prop.Vc = V_chamber(Ind);
rocket.prop.Ac = A_chamber(Ind);
rocket.prop.tc = t_chamber(Ind);
rocket.prop.m_engine = m_engine(Ind);
rocket.prop.ID = ID(Ind);
rocket.prop.OD = OD(Ind);
rocket.prop.Lfrustum = L1(Ind);

rocket.prop.Me = Me(Ind);
rocket.prop.Ae = Ae(Ind);
rocket.prop.Ln = Ln(Ind);
rocket.prop.De = De(Ind);
rocket.prop.eps = eps(Ind);

rocket.prop.m_ox = m_ox(Ind);
rocket.prop.V_ox = V_ox(Ind);
rocket.prop.L_ox = L_oxtank(Ind);
rocket.prop.m_oxtank = m_oxtank(Ind);

rocket.prop.m_fuel = m_fuel(Ind);
rocket.prop.V_fuel = V_fuel(Ind);
rocket.prop.L_fuel = L_fueltank(Ind);
rocket.prop.m_fueltank = m_fueltank(Ind);

rocket.prop.m_press = m_press(Ind);
rocket.prop.V_press = V_press(Ind);
rocket.prop.L_presstank = L_presstank(Ind);
rocket.prop.D_presstank = D/2;
rocket.prop.m_presstank = m_presstank(Ind);
%%
% figure
% hold on
% yyaxis left
% plot(OF,Isp,'LineWidth',2)
% ylabel('Specific Impulse, s')
% yyaxis right
% plot(OF,mtot,'LineWidth',2)
% xlabel('OF Ratio')
% ylabel('Propulsion System Mass, lbm')
% grid on


end

function [t,L,m] = vessel(P,D,V,Mat,FoS)
    R = D/2;  % in
    l = (V*12^3-4/3*pi*R^3)/(pi*R^2); % cylindrical length, in3
    L = l+D;   % total length, in
    SA = 2*pi*R*l+4*pi*R^2;
    t = R*P/Mat.strength*FoS;
    m = SA.*t/12^3*Mat.density;
end
