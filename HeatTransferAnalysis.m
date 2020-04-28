load('LOXCH4comb.mat','combustion');
m = Methane();
R = 10.73159; % psi*ft3/lbmol*R
PC = 350; % chamber pressure, psi
OF = 2.71;
mdot = 10; % lbm/s

% get transport and thermal properties
o = Oxygen(); mw_ox = meanMolecularWeight(o);
f = Methane(); mw_fuel = meanMolecularWeight(f);
OF_stoich = (2*mw_ox)/(mw_fuel);
phi = OF_stoich/OF;
gas = GRI30('Mix');
nsp = nSpecies(gas); % Number of Species
i_f = speciesIndex(gas,'CH4'); % get fuel index
i_o  = speciesIndex(gas,'O2'); % get ox index
x = zeros(nsp,1); % initial array of 0 mole fractions
x(i_f) = phi;
x(i_o) = 2.0; % 2 moles of oxygen per mole of fuel

p = PC/14.7*101325;
set(gas,'Temperature',300,'Pressure',p,'MoleFractions',x)
h0 = enthalpy_mass(gas); % J/kg
% get liquid oxygen and methane states
set(o,'Temperature',90,'Pressure',p);
set(f,'Temperature',111,'Pressure',p);
h_ox1 = enthalpy_mass(o);
h_fuel1 = enthalpy_mass(f);
set(o,'Temperature',300,'Pressure',p);
set(f,'Temperature',300,'Pressure',p);
h_ox2 = enthalpy_mass(o);
h_fuel2 = enthalpy_mass(f); 
hmod = h0 - (h_ox2 - h_ox1)*(OF/(OF+1)) - (h_fuel2 - h_fuel1)*(1/(OF+1));
equilibrate(gas,'HP');
set(gas,'P',p,'H',hmod)
equilibrate(gas,'HP');

s0 = entropy_mass(gas);
T0 = temperature(gas);
gam = cp_mass(gas)/cv_mass(gas); % specific heat ratio
c1 = gam + 1;
c2 = gam - 1;
c3 = c1/c2;
mw = combustion.mw(PC,OF);
cstar = sqrt(8314/mw*T0/(gam*(2/c1)^(c3)));
rho0 = density(gas)/3.28^3*2.2; % chamber density, lb/ft3

visc0 = viscosity(gas); % viscosity, P*s = N/m2*s = kg*m/s
lambda0 = thermalConductivity(gas); % therm. cond.
Pr0 = visc0*cp_mass(gas)/lambda0; % Prandtl number

At = mdot*cstar*3.28/(PC)/32.174; % in2
Pt = PC*(1+(gam-1)/2)^(-gam/(gam-1));
set(gas,'P',Pt/14.7*oneatm,'S',s0)

Tt = temperature(gas);
visc_t = viscosity(gas); % viscosity
lambda_t = thermalConductivity(gas); % W/m-Ktherm. cond.
Pr_t = visc_t*cp_mass(gas)/lambda_t; % Prandtl number
ht = enthalpy_mass(gas);
vt = sqrt(2*(hmod-ht)); % m /s
rhot = density(gas); % kg/m3

% calculate heat transfer coefficients
% at chamber
A0 = At*3;
v0 = mdot/(rho0*A0/144); % ft / s
D0 = sqrt(4*A0/pi)/12/3.28; % m
Re0 = (D0*v0/3.28*rho0*3.28^3/2.2)/(visc0);

Nu0 = 0.023*Re0^.8*Pr0^.4;
hg0 = Nu0*lambda0/D0; % W/m2-K

% at throat
Dt = sqrt(4*At/pi)/12/3.28; % m
Re_t = (Dt*vt*rhot)/(visc_t);
Nu_t = 0.023*Re_t^0.8*Pr_t^0.4;
hg_t = Nu0*lambda_t/Dt;

% get wall thickness based on hoop stress
FOS = 1.5; % chamber factor of safety
sigma_y_steel = 42100; % yield strength of 303 steel, psi
tc = PC*D0/2/(sigma_y_steel/FOS); % m
%%
Tl = 177; % K
set(m,'T',Tl,'P',PC*1.25/14.7*oneatm)


%%
k_SS = 16.26; % W / m-k
hl = 4000:10:10000;
q = (T0-Tl)./(1./hg0+tc./k_SS+1./hl);
Twg = T0 - q./hg0;
Twg_max = 1648.15*0.8;

figure
hold on
plot(hl, q)
yyaxis right
plot(hl, Twg)
yline(Twg_max,'r--','LineWidth',2);
legend('q','Twg')

%%
hl_min = ((T0-Tl)/(hg0*(T0-Twg_max))-1/hg0-tc/k_SS)^-1;
q_min = hg0*(T0-Twg_max);
Tl_arr = 100:2:177;
hl_min_arr = ((T0-Tl_arr)./(hg0.*(T0-Twg_max))-1/hg0-tc/k_SS).^-1;
figure; plot(Tl_arr, hl_min_arr)
grid on
xlabel('Liquid Coolant Temperature, K')
ylabel('Minimum Heat Transfer Coefficient Required, W/m2-K')
%%
hvap_m = 8.17*1000/mw_fuel*1000; % methane enthalpy of vaporization, J/kg
mdot_f = mdot*1/(OF+1)/2.2; % kg/s
Q_methane_vap = mdot_f*hvap_m; % W

%%
D_cool = (0.01:0.01:1)/12/3.28; % coolant passage diameter, m
set(m,'T',176,'P',350*1.25/14.7*oneatm);
cp2 = cp_mass(m); % specific heat at outlet, J/kg-K
set(m,'T',100,'P',350*1.25/14.7*oneatm); % at inlet
cp1 = cp_mass(m);
cp_m = (cp2+cp1)/2;
rho_m = density(m); % kg/m3
visc_m = .06/100; % Pa-s
lambda_m = 201E-3; % W/m-K
Pr_m = visc_m.*cp_m./lambda_m;
%v = (0.023./hl_min.*cp_m.*rho_m.^0.8.*(D_cool./visc_m).^(-.2)...
%    .*Pr_m.^(-2/3)).^(-1.25);
mdot_cool = rho_m.*(pi./4.*D_cool.^2).*(hl_min./(.023.*cp_m.*rho_m.^.8.*(D_cool./visc_m).^(-.2).*Pr_m.^(-2/3))).^(1/.8);

figure
plot(D_cool*12*3.28, mdot_cool*2.2)
xlabel('Coolant Passage Diameter, in')
ylabel('Mdot coolant Required, lbm/s')
hold on
yline(mdot_f*2.2,'--','LineWidth',2)
grid on
%% check work
% d = 0.5/12/3.28; % m
% v = 5/3.28; % ft/s
% Re_m = (d*v*rho_m/visc_m);
% hl = (.023*cp_m*rho_m*v*Re_m^(-.2)*Pr_m^(-2/3));

%%
Q_coolant_capacity = mdot_f*cp_m*(176-100);