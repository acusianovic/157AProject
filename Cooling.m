close all; clear;

%% Load data from preliminary design

load('LOXCH4comb.mat','combustion');
load('LoxTransport.mat');
load('LoxBoiling.mat');
load('betsyMK4.mat');
prop = betsyMK4.prop;

Pc = prop.PC*6894.76; % chamber pressure, Pa
cstar = prop.cstar; % characteristic velocity, m/s
OF = prop.OF; % O/F ratio
mdot = prop.mdot*0.453592; % propellant mass flow rate, kg/s
mdotl = prop.mdot_ox*0.453592; % coolant (lox) mass flow rate, kg/s


%% Reconstruct gas for later evaluations

o = Oxygen(); mw_ox = meanMolecularWeight(o);
tf = Methane(); mw_fuel = meanMolecularWeight(tf);
OF_stoich = (2*mw_ox)/(mw_fuel);
phi = OF_stoich/OF;
gas = GRI30('Mix');
nsp = nSpecies(gas); % Number of Species
x = zeros(nsp,1); % initial array of 0 mole fractions
x(speciesIndex(gas,'CH4')) = phi;
x(speciesIndex(gas,'O2')) = 2.0; % 2 moles of oxygen per mole of fuel
set(gas,'Temperature',300,'Pressure',Pc,'MoleFractions',x)
h0 = enthalpy_mass(gas); % J/kg
set(o,'Temperature',90,'Pressure',Pc);
set(tf,'Temperature',111,'Pressure',Pc);
h_ox1 = enthalpy_mass(o);
h_fuel1 = enthalpy_mass(tf);
set(o,'Temperature',300,'Pressure',Pc);
set(tf,'Temperature',300,'Pressure',Pc);
h_ox2 = enthalpy_mass(o);
h_fuel2 = enthalpy_mass(tf);
hmod = h0-(h_ox2-h_ox1)*(OF/(OF+1))-(h_fuel2-h_fuel1)*(1/(OF+1));
equilibrate(gas,'HP');
set(gas,'P',Pc,'H',hmod)
equilibrate(gas,'HP');


%% Get gas stagnation properties

gam = cp_mass(gas)/cv_mass(gas); % gas specific heat ratio
Tg0 = temperature(gas); % gas stagnation temperature, K
rhog0 = density(gas); % gas stagnation density, kg/m3
mug0 = viscosity(gas); % gas stagnation viscosity, Pa-s
cpg0 = cp_mass(gas); % gas stagnation specific heat, J/kg-K
kg0 = thermalConductivity(gas); % gas stagnation thermal conductivity, W/m-k
Prg0 = mug0*cpg0/kg0; % gas stagnation Prandtl number


%% Construct engine geometry

Vc = prop.Vc*0.0254^3; % chamber volume, m3
At = prop.At*0.0254^2; % throat area, m2
Ae = prop.Ae*0.0254^2; % exit area, m2
Ac = At*3; % chamber area, m2
theta = 30; % contraction angle, deg
alpha = 15; % expansion angle, deg
rc1 = 0.05; % frustum entry radius of curvature, m
rc2 = 0.05; % throat radius of curvature, m
dx = 0.001; % location precision, m

% find component length and generate location and diamter arrays
[x,D,Dc,Dt,De,Lc,Lf,Ln] = Geogen(Vc,At,Ae,Ac,theta,alpha,rc1,rc2,dx);

A = pi.*D.^2/4; % area array, m2
dR = [0,diff(D/2)]; % differential radius element, m
dl = sqrt(dx^2+dR.^2); % differential wall length element, m
dA = pi*dl.*(D+dR); % differential wall area element, m2

FOS = 1.5; % chamber factor of safety
sigmaSS310 = 19300; % yield strength of 310 steel, psi
tw = Pc/6894.76*Dc/2/(sigmaSS310/FOS); % chamber wall thickness, m
kSS = 17; % stainless steel thermal conductivity, W/m-k

tc = 0.001; % thermal barrier coating thickness, m
kc = 2.5; % thermal barrier coating thermal conductivity, W/m-k


%% Define coolant passage geometry

n = 120; % number of passages
Ap = 0.0002/n; % passage cross-sectional area, m2
tf = 0.5/1000; % passage wall (fin) thickness, m
a = pi.*D/n-tf; % passage width, m
b = Ap./a; % passage height, m
Dh = 2*Ap./(a+b); % passage hydraulic diameter, m


%% Determine engine mass

SA = sum(dA); % enginer inner surface area, m2
Vinner = SA*((D+tw)/D)*tw; % inner shell volume, m3
Vouter = SA*((D+3*tw)/D)*tw; % outer shell volume, m3
Vfins = sum(n*dl.*b*tf); % fins volume, m3
mengine = (Vinner+Vouter+Vfins)*7900; % engine mass, kg
% chamber entry cap/ injector not included


%% Initialize simulation

Bartz = 0.026/Dt^0.2*mug0^0.2*cpg0/Prg0^0.6*(Pc/cstar)^0.8*(Dt/rc2)^0.1; % Bartz relation constant term

% prepare area ratio - Mach number lookup table
Mdata1 = 0.01:0.01:1;
ARdata1 = ((gam+1)/2)^(-(gam+1)/(2*(gam-1)))*(1+(gam-1)/2*Mdata1.^2).^((gam+1)/(2*(gam-1)))./Mdata1;
Mdata2 = 1:0.01:4;
ARdata2 = ((gam+1)/2)^(-(gam+1)/(2*(gam-1)))*(1+(gam-1)/2*Mdata2.^2).^((gam+1)/(2*(gam-1)))./Mdata2;

% prepare Reynolds number - friction factor lookup table
Redata = 4000:1000:2e5;
syms RE F
eqn = 1/sqrt(F) == 1.930*log10(RE*sqrt(F))-0.537;
fddata = double(subs(solve(eqn,F),RE,Redata));

M = zeros(1,length(x)); % gas Mach number
Tg = zeros(1,length(x)); % gas static temperatuere, K
Taw = zeros(1,length(x)); % adiabatic wall temperature, K
Twg = zeros(1,length(x)); % coating surfact temperature, K
Twi = zeros(1,length(x)); % hot side wall temperature, K
Twl = zeros(1,length(x)); % coolant side wall temperature, K
Tb = zeros(1,length(x)); % coolant boiling point, K
Tl = zeros(1,length(x)); Tl(1) = 60; % coolant temperature, K
Pl = zeros(1,length(x)); Pl(1) = 400*6894.76; % coolant pressure, Pa
vl = zeros(1,length(x)); % coolant velocity, m/s
Rel = zeros(1,length(x)); % coolant Reynolds number
f = zeros(1,length(x)); % Darcy friction factor

hg = zeros(1,length(x)); % coolant velocity, m/s
hl = zeros(1,length(x)); % liquid heat transfer coefficient, W/m2-K
hleff = zeros(1,length(x)); % effective liquid heat transfer coefficient, W/m2-K
multi = zeros(1,length(x)); % effective coolant wetted area multiplier
U = zeros(1,length(x)); % overall heat transfer coefficient, W/m2-K
dq = zeros(1,length(x)); % heat transfer over area element, W

O2 = Oxygen();


%% Run simulation

for i = 1:length(x) % run through the entire engine
    if i < 1+(Lc+Lf)/dx
        M(i) = interp1(ARdata1,Mdata1,A(i)/At,'spline'); % gas Mach number
    else
        M(i) = interp1(ARdata2,Mdata2,A(i)/At,'spline'); % gas Mach number
    end
    Tg(i) = Tg0/(1+(gam-1)/2*M(i)^2); % gas static temperatuere, K
    set(gas,'T',Tg(i));
    mug = viscosity(gas); % gas viscosity, Pa-s
    cpg = cp_mass(gas); % gas specific heat, J/kg-K
    kg = thermalConductivity(gas); % gas thermal conductivity, W/m-k
    Prg = mug*cpg/kg; % gas Prandtl number
    r = Prg^(1/3); % recovery factor
    Taw(i) = Tg(i)+(Tg0-Tg(i))*r; % adiabatic wall temperature, K
    
    set(O2,'T',Tl(i),'P',Pl(i));
    cpl = cp_mass(O2); % coolant specific heat, J/kg-K
    rhol = density(O2); % coolant density, kg/m3
    vl(i) = mdotl/rhol/(n*Ap); % coolant velocity, m/s
    mul = LoxTransport.mu(Tl(i),Pl(i))/10^6; % coolant viscosity, Pa-s
    kl = LoxTransport.k(Tl(i),Pl(i)); % coolant thermal conductivity, W/m-K
    Prl = mul*cpl/kl; % coolant Prandtl number
    Rel(i) = rhol*vl(i)*Dh(i)/mul; % coolant Reynolds number
    
    Twg_old = (Taw(i)+Tl(i))/2; % inital guess temperature for interation
    Twl_old = (Taw(i)+Tl(i))/2; % inital guess temperature for interation
    c1 = (1+(gam-1)/2*M(i)^2); c2 = (1+(gam-1)/2*M(i)^2);
    
    for j = 1:5 % interate at each location for 5 times to converge
        hg(i) = Bartz*(At/A(i))^0.9/((Twg_old/Taw(i)*c1+1)/2)^0.68*c2^0.12; % gas heat transfer coefficient, W/m2-K
        if j == 1
            eta = 0.5; % inital guess fin efficiency
        else
            eigen = sqrt(2*hl(i)/kSS/tf); % fin equation eigenvalue
            eta = tanh(eigen*b(i))/(eigen*b(i)); % fin efficiency
        end
        multi(i) = (a(i)/(a(i)+tf)+2*b(i)/(a(i)+tf)*eta); % effective coolant wetted area multiplier
        hl(i) = 0.0185*Rel(i)^0.8*Prl^0.4*(Tl(i)/Twl_old)^0.1*kl/Dh(i); % liquid heat transfer coefficient, W/m2-K
        hleff(i) = hl(i)*multi(i); % effective liquid heat transfer coefficient, W/m2-K
        U(i) = 1/(1/hg(i)+tc/kc+tw/kSS+1/hleff(i)); % overall heat transfer coefficient, W/m2-K
        dq(i) = (Taw(i)-Tl(i))*U(i)*dA(i); % heat transfer over area element, W
        Twg_old = Taw(i)-dq(i)/hg(i)/dA(i);
        Twl_old = Tl(i)+dq(i)/hleff(i)/dA(i);
    end
    Twg(i) = Twg_old; % coating surface temperature, K
    Twl(i) = Twl_old; % liquid side wall temperatue, K
    Twi(i) = Twg(i)-dq(i)/(kc/tc)/dA(i); % hot side wall temperature, K
    Tb(i) = LoxBoiling.T(Pl(i)); % coolant boiling point, K
    f(i) = interp1(Redata,fddata,Rel(i),'spline'); % Darcy friction factor
    dp = dl(i)*f(i)*rhol/2*vl(i)^2/Dh(i); % coolant pressure drop, Pa
    if i < length(x)
        Tl(i+1) = dq(i)/mdotl/cpl+Tl(i); % increased coolant temperature, K
        Pl(i+1) = Pl(i)-dp; % decreased coolant pressure, Pa
    end
end


%% Plot results

figure
set(gcf,'defaultlinelinewidth',2,'defaultaxesfontsize',14)
yyaxis left
hold on
plot(x,Twg,'-','color','#EDB120')
plot(x,Twi,'-','color','#77AC30')
plot(x,Twl,'-','color','#4DBEEE')
plot(x,Tb+50,'--','color','#4DBEEE')
plot(x,Tl,'-','color','#0072BD')
plot(x,Tb,'--','color','#0072BD')
hold off
ylabel('Temperature (K)')
yyaxis right
plot(x,D)
ylabel('Engine inner diameter (m)')
xlabel('Downstream location (m)')
legend('Coating surface','Wall surface (hot)','Wall surface (cold)','Film boiling limit','Coolant','Boiling point','location','northeast')

figure
set(gcf,'defaultlinelinewidth',2,'defaultaxesfontsize',14)
yyaxis left
plot(x,Taw,x,Tg)
yline(Tg0,'linewidth',2,'color','#A2142F');
ylabel('Temperature (K)')
yyaxis right
plot(x,M)
ylabel('Mach number')
xlabel('Downstream location (m)')
legend('Adiabatic wall temperature','Gas static temperature','Stagnation temperature')

figure
set(gcf,'defaultlinelinewidth',2,'defaultaxesfontsize',14)
yyaxis left
plot(x,Pl/6894.76)
ylabel('Coolant pressure (psia)')
yyaxis right
plot(x,f)
ylabel('Darcy friction factor')

figure
set(gcf,'defaultlinelinewidth',2,'defaultaxesfontsize',14)
yyaxis left
plot(x,Dh*1000,x,a*1000,x,b*1000)
ylabel('Passage dimensions (mm)')
yyaxis right
plot(x,b./a)
ylabel('Aspect ratio')
xlabel('Downstream location (m)')
legend('Hydraulic diameter','Passage width','Passage height')

figure
set(gcf,'defaultlinelinewidth',2,'defaultaxesfontsize',14)
yyaxis left
plot(x,multi)
ylabel('Effective coolant wetted area multiplier')
yyaxis right
plot(x,b./a)
ylabel('Aspect ratio')
xlabel('Downstream location (m)')

function [x,D,Dc,Dt,De,Lc,Lf,Ln] = Geogen(Vc,At,Ae,Ac,theta,alpha,rc1,rc2,dx)
    Dt = sqrt(4*At/pi); % throat diameter, m
    De = sqrt(4*Ae/pi); % exit diameter, m
    Dc = sqrt(4*Ac/pi); % chamber diameter, m

    Lf = (Dc-Dt)/2*tand(theta); % frustum length (first order estimate), m
    Lc = (Vc-Ac*Lf*(1+sqrt(At/Ac)+At/Ac)); % chamber length, m

    Lc = round(Lc,3); % rounded chamber length, m
    x0 = Lc;
    x1 = round(x0+rc1*sind(theta),3); y1 = Dc/2-rc1*(1-cosd(theta));
    y2 = Dt/2+rc2*(1-cosd(theta)); x2 = round(x1+(y1-y2)/tand(theta),3);
    xt = x2+rc2*sind(theta);
    x3 = round(xt+rc2*sind(alpha),3); y3 = Dt/2+rc2*(1-cosd(alpha));
    ye = De/2; xe = round(x3+(ye-y3)/tand(alpha),3);
    Lf = xt-x0; % frustum length, m
    Ln = xe-xt; % nozzle length, m

    x = 0:dx:xe; % downstream location array, m
    xarray1 = 0:dx:x0;
    xarray2 = x0:dx:x1;
    xarray3 = x1:dx:x2;
    xarray4 = x2:dx:x3;
    xarray5 = x3:dx:xe;
    yarray1 = Dc/2*ones(1,length(xarray1));
    yarray2 = Dc/2-rc1+sqrt(rc1^2-(xarray2-x0).^2);
    yarray3 = linspace(y1,y2,length(xarray3));
    yarray4 = Dt/2+rc2-sqrt(rc2^2-(xarray4-xt).^2);
    yarray5 = linspace(y3,De/2,length(xarray5));
    D = 2*[yarray1,yarray2(2:end),yarray3(2:end),yarray4(2:end),yarray5(2:end)]; % engine diameter array, m
end