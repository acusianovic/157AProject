clc; clear all; close all;
Itot = 200000; % lbf-sec
tb = 40; % sec
T = Itot/tb; % lbf
P0 = linspace(200,500,201); % psia
ga = 1.145; % gamma
cstar = 6015;
Pe = 7.663; % psia
Cf = sqrt(2*ga^2/(ga-1)*(2/(ga+1))^((ga+1)/(ga-1))*(1-(Pe./P0).^((ga-1)/ga)));
Eff = 0.95;
At = Itot./(P0.*Cf*Eff*tb); % in2
Isp = cstar.*Cf*Eff/32.174; % sec
mdot = P0.*At/cstar*32.174; % lbm/sec
mprop = mdot*tb; % lbm
OFRatio = 2.71;
LO2.Mass = mprop*OFRatio/(1+OFRatio); % lbm
LCH4.Mass = mprop*1/(1+OFRatio); % lbm
LO2.Density = 71.2; % lbm/ft3
LCH4.Density = 26.4; % lbm/ft3
OxTank.Volume = LO2.Mass./LO2.Density; % ft3
FuelTank.Volume = LCH4.Mass./LCH4.Density; % ft3
Ptank = P0*1.25+70; % psia
Al6061 = struct('Strength',40000,'Density',168.6); % psia, lbm/ft3
SS310 = struct('Strength',19300,'Density',492.6);  % psia, lbm/ft3
FoS = 1.5;

Dtank = 10; % in
[OxTank.Thickness,OxTank.Length,OxTank.SurfaceArea,OxTank.Mass] = vessel(Ptank,Dtank,OxTank.Volume,Al6061,1.5);
[FuelTank.Thickness,FuelTank.Length,FuelTank.SurfaceArea,FuelTank.Mass] = vessel(Ptank,Dtank,FuelTank.Volume,Al6061,1.5);
% in, in, in2, lbm

Dchamber = 8; %in
Lstar = 50; % dim
Chamber.Volume = Lstar.*At/12^3; % ft3
[Chamber.Thickness,Chamber.Length,Chamber.SurfaceArea,Chamber.Mass] = vessel(P0,Dchamber,Chamber.Volume,SS310,1.5);
% in, in, in2, lbm

mplumbing = 10;
mdry = OxTank.Mass + FuelTank.Mass + Chamber.Mass + mplumbing;
mwet = LO2.Mass + LCH4.Mass + OxTank.Mass + FuelTank.Mass + Chamber.Mass + mplumbing;
[mwet,i] = min(mwet);
mdry = mdry(i);
mprop = mprop(i);
LO2.Mass = LO2.Mass(i);
LCH4.Mass = LCH4.Mass(i);

P0 = P0(i);
Isp = Isp(i);
mdot = mdot(i);
At = At(i);
Cf = Cf(i);

Ptank = Ptank(i);
OxTank = best(OxTank,i);
FuelTank = best(FuelTank,i);
Chamber = best(Chamber,i);

Ltot = (OxTank.Length + FuelTank.Length + Chamber.Length)/12; % ft

fprintf('Optimum comfiguration: \n')
fprintf('Chamber pressure:    %d psia \n',P0)
fprintf('Isp:                 %.1f sec \n',Isp)
fprintf('Wet mass:            %.1f lb \n',mwet)
fprintf('Propellant mass:     %.1f lb \n',mprop)
fprintf('  Oxidizer mass:     %.1f lb \n',LO2.Mass)
fprintf('  Fuel mass:         %.1f lb \n',LCH4.Mass)
fprintf('Dry mass:            %.1f lb \n',mdry)
fprintf('  OxTank mass:       %.1f lb \n',OxTank.Mass)
fprintf('  FuelTank mass:     %.1f lb \n',FuelTank.Mass)
fprintf('  Chamber mass:      %.1f lb \n',Chamber.Mass)
fprintf('  Plumbing mass:     %.1f lb \n',mplumbing)



function [t,L,SA,m] = vessel(P,D,V,Mat,FoS)
    R = D/2;
    l = (V*12^3-4/3*pi*R^3)/(pi*R^2);
    L = l+D;
    SA = 2*pi*R*l+4*pi*R^2;
    t = R*P/Mat.Strength*FoS;
    m = SA.*t/12^3*Mat.Density;
end
function Vessel = best(Vessel,i)
    Vessel.Volume = Vessel.Volume(i);
    Vessel.Mass = Vessel.Mass(i);
    Vessel.Length = Vessel.Length(i);
    Vessel.Thickness = Vessel.Thickness(i);
    Vessel.SurfaceArea = Vessel.SurfaceArea(i);
end