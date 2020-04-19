%2D Trajectory Code (in progress)
%Main Author: JP

clear variables;close all;clc

%Load Atmospheric Model
load('AtmosphericPressureModel.mat', 'PressureAltitude','AtmosphericPressure');
load('AtmosphericDensityModel.mat', 'DensityAltitude','Density');

%%% Rocket Geometry %%%
RocketDiam = 1.25; %diameter [inches]
NozzleArea = 86.800174; %nozzle exit area [in^2]
StructuralMass = 150;%[lbm]

%%% Tank Info %%%

%%% Propulsion Parameters
TotalImpulse = 200000; %[lbf-s]
BurnTime = 40; %[s]
SHR= 1.145; %specific heat ratio
cstar = 6015;
Pe = 7.663; %exit pressure [psia]
Cf = sqrt(2*SHR^2/(SHR-1)*(2/(SHR+1))^((SHR+1)/(SHR-1))*(1-(Pe./P0).^((SHR-1)/SHR))); %thrust coefficient
Eff = 0.95; %nozzle efficiency
Isp = Eff*(cstar.*Cf/32.174); %sea level Isp [s]
At = Itot./(P0.*Cf*Eff*tb); %throat area [in^2]
mdot = P0.*At/cstar*32.174; %mass flow rate [lbm/sec]

%%% Trajectory Initial Conditions %%%
t0 = 0; %time [s]
h0 = 0; %altitude [ft]
x0 = 0; %drift [ft]
vx0 = 0; %drift velocity [fts]
vy0 = 0; %vertical velocity [ft/s] 

%%% Loop Parameters %%%
dt = 0.1; %time step [s]
Iterations = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

while x0 >= 0 && Iterations <= MaxIterations
    
    
end

