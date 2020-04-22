%2D Trajectory Code (in progress)
%Main Author: JP

clear variables;close all;clc

%%% Load Atmospheric Model %%%
load('AtmosphericPressureModel.mat', 'PressureAltitude','AtmosphericPressure');
load('AtmosphericDensityModel.mat', 'DensityAltitude','Density');
PressureAltitude = 3.28084*PressureAltitude; %[ft]
DensityAltitude = 3.28084*DensityAltitude; %[ft]
AtmosphericPressure = 0.000145038*AtmosphericPressure; %[psi]
Density = 0.00194032*Density; %[slug/ft^3]

%%% Physical Parameters %%%
g0 = 32.174; %sea level gravitational acceleration [ft/s^2]
Pa = 14.7; %sea level atmsopheric pressure [psia]

%{
%%% Rocket Geometry %%%
RocketDiam = 1.25; %diameter [inches]
NozzleExitArea = 86.800174; %nozzle exit area [in^2]
StructuralMass = 150;%[lbm]

%%% Propulsion Parameters %%%
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
OFRatio = 2.71;
LO2.Mass = mprop*OFRatio/(1+OFRatio); %[lbm]
LCH4.Mass = mprop*1/(1+OFRatio); %[lbm]
LO2.Density = 71.2; % [lbm/ft^3]
LCH4.Density = 26.4; % [lbm/ft3^3]

%%% Tank Info %%%%
FuelMass = LO2.Mass+LCH4.Mass; %[lbm]
OxTank.Volume = LO2.Mass./LO2.Density; % [ft^3]
FuelTank.Volume = LCH4.Mass./LCH4.Density; % [ft^3]
Ptank = P0*1.25+70; % [psia]
DryMass = StructuralMass+TankMass;

%%% Trajectory Initial Conditions %%%
Ft = mdot*Isp*g0+(Pe-Pa)*NozzleArea;
Fg = (DryMass+FuelMass)*g0;
LaunchAngle = 4*pi/180; [rad]
%}
t = 0; %time [s]
h = 0; %altitude [ft]
x = 0; %drift [ft]
vx = 0; %drift velocity [ft/s]
vy = 0; %vertical velocity [ft/s] 
v = sqrt(vx^2+vy^2); %velocity mag [ft/s]
ax = 0; %x acceleration [ft/s^2]
ay =0; %y acceleration [ft/s^2]

%%% Aerobee 150 Benchmark Override %%%
FuelMass = 1073.7/32.2; % [slugs] 862kg
DryMass = 278.15/32.2; % [slugs] 68kg PL
m = DryMass+FuelMass; %wet mass [slugs]

RocketDiam = 1.25; %[ft] 0.38m
Lt = 303; %total length [in]
Sb = pi*RocketDiam*Lt; %body surface area [in^2]
Cr = 39; %fin root chord [in]
Ct = 27; %fin tip chord [in]
FinThick = 1.1; %fin thickness [in]
Nf = 4;%number of fins
Sf = (16.1/2)*(Cr+Ct);
Lp = 0.75; %launch lug length [in]
aL = 262; %nose to launch lug length [in]
APro = Lp*0.375; %maximum cross section area of launch lug [in^2]
Spro = 3*0.75+2*0.375; %wetted surface area of proturbance [in^2]
LN = 87.8; %nose length [in]
Lb = Lt-LN; %length of the body [in]

Ft = 4100; %liftoff thrust[lbf] 18kN
Isp = 198; %[s]
mdot = Ft/(g0*Isp); %[slugs/s]
NozzleExitArea = 0.296875; %[ft^2]
LaunchAngle = 0*pi/180; %[rad]
AOA = LaunchAngle; %angle of attack[rad]
ChamberPressure = 324; %[psia]
Gamma = 1.145; %specific heat ratio
NER = 4.6; %nozzle expansion ratio 
Pe = Pa; %assume perfectly expanded at sea level

%%% Loop Parameters %%%
dt = 0.1; %time step [s]
step = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

%counters
ThrustCounter = 0; %for finding burnout parameters later

while h(step) >= 0 && step <= MaxIterations
    
    %%% Forces %%%
    
    %Thust
    if m(step) > DryMass %during burn
        %update ambient pressure
        Pa = interp1(PressureAltitude,AtmosphericPressure,h(step));%[psia]
        %update thrust
        Ft(step+1) = mdot*g0*Isp+(Pe-Pa)*NozzleExitArea; %[lbf]
        %update mass
        m(step+1) = m(step)-mdot*dt; %[slugs]
        %update thrust counter
        ThrustCounter = ThrustCounter+1;
    else %after burn
        Ft(step+1) = 0;
        m(step+1) = m(step);
    end
    
    %Gravitational Force
    %update gravitational acceleration
    g = g0*((2.0856*10^7)/(2.0856*10^7+h(step)))^2;
    %calculate new force due to gravity
    Fg = -m(step)*g;
    
    %Drag Force
    %get local air density
    rhoAir = interp1(DensityAltitude,Density,h(step)); %[slugs/ft^3]
    if v(step) == 0 %not moving
        Fd = 0;
    elseif vy(step) > 0 %ascent
        Af = (pi/4)*RocketDiam^2;
        [ Cd(step), Mach(step),FricDrag(step) ] = GetCd( h(step), v(step), Lt, RocketDiam*12, Cr, Ct, Nf, Sf, Sb,...
            FinThick, Lp, aL, APro, Spro, LN, Lb,step ); %convert diameter to inches
        Fd = -0.5*rhoAir*v(step)^2*Cd(step)*Af;
    else %Descent
        Af = (pi/4)*RocketDiam^2;
        Fd = 0.5*rhoAir*v(step)^2*1.5*Af;
    end
    %
    
    %%% Kinematics %%%
    
    %calculate net force in each direction
    Fx = (Ft(step)+Fd)*sin(AOA);
    Fy = (Ft(step)+Fd)*cos(AOA)+Fg;
    %acceleration
    ax(step+1) = Fx/m(step);
    ay(step+1) = Fy/m(step);
    %velocity
    vx(step+1) = vx(step)+ax(step)*dt;
    vy(step+1) = vy(step)+ay(step)*dt;
    v(step+1) = sqrt(vx(step+1)^2+vy(step+1)^2);
    %position
    x(step+1) = x(step)+vx(step)*dt;
    h(step+1) = h(step)+vy(step)*dt;
    if h(step+1) <= 0 && x(step) == 0 %we haven't lifted off yet
        x(step+1) = 0;
        h(step+1) = 0;
    end
    
    %update time
    t(step+1) = t(step)+dt;
    
    %update counter
    step = step+1;
    
end



