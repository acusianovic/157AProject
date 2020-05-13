function combustion = getCombustion(ox,fuel,PC,OF)
%% Inputs
% Ox: string of liquid oxidizer name. ex 'O2'
% Fuel: String of liquid fuel name. ex 'CH4'
% PC: Array of chamber pressures
% OF: Array of OF
T = zeros(length(PC),length(OF));
gam = zeros(length(PC),length(OF));
cp = zeros(length(PC),length(OF));
mw = zeros(length(PC),length(OF));
cstar = zeros(length(PC),length(OF));
optOF = zeros(length(PC),1);
optcstar = zeros(length(PC),1);
%% Get equivalence ratio from mixture ratio
o = Solution('nasa_gas.cti');
set(o,'X',char(ox+':1'));
mw_ox = meanMolecularWeight(o);
f = Solution('nasa_gas.cti');
set(f,'X',char(fuel+":1"));
mw_fuel = meanMolecularWeight(f);

OF_stoich = (2*mw_ox)/(mw_fuel);
phi_arr = OF_stoich./OF;

%% Equilibrate for range of phi and pressure
gas = Solution('nasa_gas.cti'); % create gas solution
nsp = nSpecies(gas); % Number of Species
i_f = speciesIndex(gas,char(fuel)); % get fuel index
i_o  = speciesIndex(gas,char(ox)); % get ox index

oxLiquid = Oxygen();
fuelLiquid = Methane();

for j = 1:length(PC)
    for k = 1:length(phi_arr)
        of = OF(k);
        p = PC(j)/14.7*101325; % Pa
        
        x = zeros(nsp,1); % initial array of 0 mole fractions
        x(i_f) = phi_arr(k);
        x(i_o) = 2.0; % 2 moles of oxygen per mole of fuel
        set(gas,'Temperature',300,'Pressure',p,'MoleFractions',x)
        h0 = enthalpy_mass(gas); % J/kg
        % get liquid oxygen and methane states
        set(oxLiquid,'Temperature',90,'Pressure',p);
        set(fuelLiquid,'Temperature',111,'Pressure',p);
        h_ox1 = enthalpy_mass(oxLiquid);
        h_fuel1 = enthalpy_mass(fuelLiquid);
        set(oxLiquid,'Temperature',300,'Pressure',p);
        set(fuelLiquid,'Temperature',300,'Pressure',p);
        h_ox2 = enthalpy_mass(oxLiquid);
        h_fuel2 = enthalpy_mass(fuelLiquid); 
        hmod = h0 - (h_ox2 - h_ox1)*(of/(of+1)) - (h_fuel2 - h_fuel1)*(1/(of+1));
        equilibrate(gas,'HP');
        set(gas,'P',p,'H',hmod)
        equilibrate(gas,'HP');
        
        T(j,k) = temperature(gas);
        gam(j,k) = cp_mass(gas)/cv_mass(gas);
        c1 = gam(j,k)+1;
        c2 = gam(j,k)-1;
        c3 = c1/c2;
        mw(j,k) = meanMolecularWeight(gas);
        cstar(j,k) = sqrt(8314/mw(j,k)*T(j,k)/(gam(j,k)*(2/c1)^(c3)));
        
    end
    [optcstar(j), ind] = max(cstar(j,:));
    optOF(j) = OF(ind);
    
    
end
%%
fprintf("Done");
figure
plot(PC/14.7, T)
grid on

%%
figure
plot(OF, cstar,'LineWidth',2)
grid on

%% 3d plot

x = PC;
y = OF;
z = cstar;

figure
hold on
[Xq,Yq] = meshgrid(x, y);
meshc(Xq',Yq',z)
xlabel('Chamber Pressure psi')
ylabel('Mixture Ratio')
zlabel('Cstar m/s')
%shading interp
set(gca, 'FontSize', 12, 'FontWeight', 'bold')
plot3(PC,optOF,optcstar,'r','LineWidth',2)

%%
figure
plot(PC,optOF,'LineWidth',2)
CEAopt.PC = PC;
CEAopt.OF = OF;
CEAopt.cstar = cstar;

%%
figure
plot(PC./14.7,gam(:,2))
%%
figure
plot(OF,cstar)

%%
combustion.cstar = griddedInterpolant({PC,OF},cstar,'spline');
combustion.T = griddedInterpolant({PC,OF},T,'spline');
combustion.mw = griddedInterpolant({PC,OF},mw,'spline');
combustion.gam = griddedInterpolant({PC,OF},gam,'spline');



end