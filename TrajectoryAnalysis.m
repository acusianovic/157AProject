%2D Trajectory Code (in progress)
%Main Author: JP

clear variables;close all;clc

%%% Load Atmospheric Model %%%
load('AtmosphericPressureModel.mat', 'PressureAltitude','AtmosphericPressure');
load('AtmosphericDensityModel.mat', 'DensityAltitude','Density');
%convert to english units
PressureAltitude = 3.28084*PressureAltitude; %[ft]
DensityAltitude1 = 3.28084*DensityAltitude; %[ft]
AtmosphericPressure = 0.000145038*AtmosphericPressure; %[psi]
Density = 0.00194032*Density; %[slug/ft^3]
%interpolate data
DensityAltitude = linspace( min(DensityAltitude1), max(DensityAltitude1), 10^5);
Density = interp1(DensityAltitude1,Density,DensityAltitude,'pchip');

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
L = 303; %total length [in]
Sb = pi*(RocketDiam*12)*L; %body surface area [in^2]
Cr = 39; %fin root chord [in]
Ct = 27; %fin tip chord [in]
tc = 1.1; %fin thickness [in]
nf = 4;%number of fins
Sf = (16.13/2)*(Cr+Ct); %fin surface area [in^2]
Lp = 0.75; %launch lug length [in]
Lap = 262; %nose to launch lug length [in]
Ap = Lp*0.375; %maximum cross section area of launch lug [in^2]
Sp = 3*0.75+2*0.375; %wetted surface area of proturbance [in^2]
Ln = 87.8; %nose length [in]
L0 = L-Ln; %length of the body [in]
xTc = 0;
db = RocketDiam*12; %[in] 0.38m

Ft = 4100; %liftoff thrust[lbf] 18kN
Isp = 198; %[s]
mdot = Ft/(g0*Isp); %[slugs/s]
NozzleExitArea = 42.75; %[in^2]
LaunchAngle = 4*pi/180; %[rad]
AOA = LaunchAngle; %angle of attack[rad]
ChamberPressure = 324; %[psia]
NER = 4.6; %nozzle expansion ratio 
Pe = Pa; %assume perfectly expanded at sea level

RecoveryAltitude = 200000; %[ft]

%%% Loop Parameters %%%
dt = 0.1; %time step [s]
step = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

%counters
ThrustCounter = 0; %for finding burnout parameters later
ChuteDeployed = 0;

while h(step) <= 300000 && step <= MaxIterations
    
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

    %Dynamic drag model
    rhoAir = interp1(DensityAltitude,Density,h(step)); %[slugs/ft^3]
    
    if vy(step) == 0 %pre-launch
        Af = (pi/4)*RocketDiam^2;
        Cd(step) = 0;
        Sign = 1;
    elseif vy(step) > 0 %before apogee
        [Cd(step),Mach(step)] = Drag(h(step),L,Ct,Cr,xTc,tc,nf,Sp,Lap,Ap,db,L0,Ln,RocketDiam*12,v(step),Sb,Sf,Lp);
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = -1;
    elseif vy(step) < 0 && h(step) > RecoveryAltitude && ChuteDeployed == 0 %after apogee, before chute deployment
        [Cd(step),Mach(step)] = Drag(h(step),L,Ct,Cr,xTc,tc,nf,Sp,Lap,Ap,db,L0,Ln,RocketDiam*12,v(step),Sb,Sf,Lp);
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = 1;
    else %thereafter: chute out, change sign depending on direction until balance
        if ChuteDeployed == 0
        ChuteDeployed = 1;
        fprintf( 'Main parachute deployed at %f s and %f ft', t(step), h(step))
        end
        Af = (pi/4)*24^2; %[ft^2]
        Cd(step) = Drag(h(step),L,Ct,Cr,xTc,tc,nf,Sp,Lap,Ap,db,L0,Ln,RocketDiam*12,v(step),Sb,Sf,Lp);
        Sign = (-vy(step)/abs(vy(step)));
    end
    Fd = Sign*0.5*rhoAir*v(step)^2*Af*Cd(step);
    

    %Simple drag model
    %{
    if v(step) == 0 && ChuteDeployed == 0 %pre-launch
        Af = (pi/4)*RocketDiam^2;
        Cd(step) = 0;
        Sign = 1;
    elseif vy(step) > 0  && ChuteDeployed == 0 %before apogee
        Cd(step) = 0.5;
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = -1;
    elseif vy(step) < 0 && h(step) > RecoveryAltitude && ChuteDeployed == 0 %after apogee, before chute deployment
        Cd(step) = 0.5;
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = 1;
    else
        if ChuteDeployed == 0
        ChuteDeployed = 1;
        fprintf( 'Main parachute deployed at %f s and %f ft', t(step), h(step))
        AOA = 0;
        end
        Af = (pi/4)*30^2; %[ft^2]
        Cd(step) = 1.75;
        Sign = (-vy(step)/abs(vy(step)));
    end
    Fd = Sign*0.5*rhoAir*v(step)^2*Af*Cd(step);
    %}

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
    if h(step+1) <= 0 && step < 100 %we haven't lifted off yet
        x(step+1) = 0;
        h(step+1) = 0;
    end
    
    %update time
    t(step+1) = t(step)+dt;
    
    %update counter
    step = step+1;
    
end

%notable outputs
fprintf('\nThe burnout time is %f s', t(ThrustCounter))
%%
figure
%sgtitle('Aerobee 150A, constant drag coefficients')
grid on
subplot(3,1,1)
plot(t,h)
title('Altitude vs. Time')
xlabel('[s]')
ylabel('[ft]')
subplot(3,1,2)
plot(t,vy)
title('Verticle Velocity vs. Time')
xlabel('[s]')
ylabel('[ft/s]')
xlim([0 t(ThrustCounter)+10])
ylim([0 vy(ThrustCounter)+1000])
subplot(3,1,3)
plot(t,ay)
xlim([0 t(ThrustCounter)+10]) 
ylim([-100 800])
title('Vertical Acceleration vs. Time')
xlabel('[s]')
ylabel('[ft/s^2]')
%%
figure
plot(Mach,Cd)
ylim([0 4])
ylabel('Cd')
xlabel('Time')

