%2D Trajectory Code (in progress)
%Main Author: JP

clear variables;close all;clc

%%% Load Atmospheric Model %%%
load('AtmosphericPressureModel.mat', 'PressureAltitude','AtmosphericPressure');
load('AtmosphericDensityModel.mat', 'DensityAltitude','Density');

%%% Physical Parameters %%%
g0 = 32.174; %sea level gravitational acceleration [ft/s^2]
Pa = 14.7; %sea level atmsopheric pressure [psia]

%%% Rocket Geometry %%%
RocketDiam = 1.25; %diameter [inches]
NozzleArea = 86.800174; %nozzle exit area [in^2]
StructuralMass = 150;%[lbm]

%%% Tank Info %%%

%%% Propulsion Parameters
TotalImpulse = 200000; %[lbf-s]
tb = 40; %burn time [s]
SHR= 1.145; %specific heat ratio
cstar = 6015;
Pe = 7.663; %exit pressure [psia]
Cf = sqrt(2*SHR^2/(SHR-1)*(2/(SHR+1))^((SHR+1)/(SHR-1))*(1-(Pe./P0).^((SHR-1)/SHR))); %thrust coefficient
Eff = 0.95; %nozzle efficiency
Isp = Eff*(cstar.*Cf/g0); %sea level Isp [s]
At = Itot./(P0.*Cf*Eff*tb); %throat area [in^2]
mdot = (P0.*At/cstar)*g0; %mass flow rate [lbm/sec]
mprop = mdot*tb; %propellant mass [lbm]

%%% Trajectory Initial Conditions %%%
t0 = 0; %time [s]
h0 = 0; %altitude [ft]
x0 = 0; %drift [ft]
vx0 = 0; %drift velocity [fts]
vy0 = 0; %vertical velocity [ft/s] 
Ft = mdot*Isp*g0+(Pe-Pa)*NozzleArea;

%%% Loop Parameters %%%
dt = 0.1; %time step [s]
Iterations = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

while x0 >= 0 && Iterations <= MaxIterations
    
    
end

