%2D Trajectory Code (in progress)
%Main Author: JP

clear variables;close all;clc

%%% Load Atmospheric Model %%%
load('AtmosphericPressureModel.mat', 'PressureAltitude','AtmosphericPressure');
load('AtmosphericDensityModel.mat', 'DensityAltitude','Density');

%convert to english units
PressureAltitude = 3.28084*PressureAltitude; %[ft]
DensityAltitudeSparce = 3.28084*DensityAltitude; %[ft]
AtmosphericPressure = 0.000145038*AtmosphericPressure; %[psi]
Density = 0.00194032*Density; %[slug/ft^3]
%interpolate data
DensityAltitude = linspace( min(DensityAltitudeSparce), max(DensityAltitudeSparce), 10^5);
Density = interp1(DensityAltitudeSparce,Density,DensityAltitude,'pchip');

%%% Load Aerobee Drag Data %%%
load('Aerobee150ADragData.mat','Aerobee150ADragData');

%%% Physical Parameters %%%
g0 = 32.174; %sea level gravitational acceleration [ft/s^2]
Pa = 14.7; %sea level atmsopheric pressure [psia]
LRL = 160; %launch rail length [ft]

t = 0; %time [s]
h = 0; %altitude [ft]
x = 0; %drift [ft]
vx = 0; %drift velocity [ft/s]
vy = 0; %vertical velocity [ft/s] 
v = sqrt(vx^2+vy^2); %velocity mag [ft/s]
ax = 0; %x acceleration [ft/s^2]
ay =0; %y acceleration [ft/s^2]

%%% Aerobee 150 Benchmark Override %%%
FuelMass = 950/32.2; % [slugs] 862kg  %950 1073.7
DryMass = 278.15/32.2; % [slugs] 68kg PL
m = DryMass+FuelMass; %wet mass [slugs]

RocketDiam = 1.25; %[ft] 0.38m
L = 303; %total length [in]
Sb = pi*(RocketDiam*12)*L; %body surface area [in^2]
Cr = 39; %fin root chord [in]
Ct = 27; %fin tip chord [in]
tc = 1.1; %fin thickness [in]
nf = 4;%number of fins
Sf = (16.13)*(Cr+Ct)/2; %fin surface area [in^2]
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

%%% Set altitude for recovery deployment %%%
RecoveryAltitude = 90000; %[ft]
ADrogue = 25; %[ft]
AChute = 100; %[ft]

%%% Loop Parameters %%%
dt = 0.01; %time step [s]
step = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

%counters
ThrustCounter = 0; %for finding burnout parameters later
DrogueDeployed = 0;
ChuteDeployed = 0;
ApogeeCounter = 0; %for finding apogee later

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
    Fg(step) = -m(step)*g;

    %Drag Force
    rhoAir = interp1(DensityAltitude,Density,h(step)); %[slugs/ft^3]
    %{
    %Dynamic drag model
    %[Cd(step),~] = Drag(h(step),L,Ct,Cr,xTc,tc,nf,Sp,Lap,Ap,db,L0,Ln,... 
    %RocketDiam*12,v(step),Sb,Sf,Lp); call dynamic Cd
    if h < 37000           
        SOS = -0.004*h(step) + 1116.45;
    elseif h <= 64000
        SOS = 968.08;
    else
        SOS = 0.0007*h(step) + 924.99;
    end
    Cd(step) = interp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),v(step)/SOS);
    
    %Aerobee Drag Model
    if vy(step) > 0 %before apogee
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = -1;
        ApogeeCounter = step;
    elseif vy(step) < 0 && h(step) > RecoveryAltitude && ChuteDeployed == 0 %after apogee, before chute deployment
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = 1;
        AOA = 0;
    else %thereafter: chute out, change sign depending on direction until balance
        if ChuteDeployed == 0
        ChuteDeployed = 1;
        fprintf( '\n Main parachute deployed at %f s and %f ft', t(step), h(step))
        end
        Af = (pi/4)*30^2; %[ft^2]
        AOA = 0;
        Sign = 1;%(-vy(step)/abs(vy(step)));
    end
    Fd(step) = Sign*0.5*rhoAir*v(step)^2*Af*Cd(step);
    %override negligible densities
    if h(step) >= 200000 && ChuteDeployed == 0
       Cd(step) = 0;
       Fd(step) = 0;
    end
    %}
    %Speed of sound
    if h(step) < 37000           
        SOS = -0.004*h(step) + 1116.45;
    elseif h(step) <= 64000
        SOS = 968.08;
    else
        SOS = 0.0007*h(step) + 924.99;
    end
    %Simple drag model (Aerobee Data)
    if v(step) == 0 && ChuteDeployed == 0 %pre-launch
        Af = (pi/4)*RocketDiam^2;
        Cd(step) = 0;
        Fd(step) = -1*0.5*rhoAir*v(step)^2*Af*Cd(step);
    elseif vy(step) > 0  && ChuteDeployed == 0 %before apogee
        %Cd(step) = 0.4;
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Cd(step) = interp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),v(step)/SOS);
        Fd(step) = -1*0.5*rhoAir*v(step)^2*Af*Cd(step);
    elseif vy(step) < 0 && h(step) > RecoveryAltitude  %deploy drogue at apogee
        Cd(step) = 0.75;
        Af = ADrogue; %[ft^2]
        if DrogueDeployed == 0
            DrogueDeployed = 1;
            fprintf( '\nDrogue deployed at %0.0fs and %0.0fft', t(step), h(step))
        end
        Sign = 1;
        AOA = 0;
        Fd(step) = -1*0.5*rhoAir*vy(step)^2*Af*Cd(step);
    else
        if ChuteDeployed == 0
            ChuteDeployed = 1;
            fprintf( '\nMain parachute deployed at %0.0fs and %0.0fft', t(step), h(step))
        AOA = 0;
        end
        Af = AChute; %[ft^2]
        Cd(step) = 1;
        Sign = (-vy(step)/abs(vy(step)));
        Fd(step) = Sign*0.5*rhoAir*vy(step)^2*Af*Cd(step);
    end
    

    %%% Kinematics %%%
    
    %calculate net force in each direction
    Fx = (Ft(step)+Fd(step))*sin(AOA);
    Fy = (Ft(step)+Fd(step))*cos(AOA)+Fg(step);
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
    
    %%% find OTRS %%%
    if sqrt(x(step)^2+h(step)^2) <= LRL
        OTRS = v(step);
        OTRSPosition = step;
    end
    
    %update time
    t(step+1) = t(step)+dt;
    
    %update counter
    step = step+1;
    
end

%notable outputs
fprintf('\nBurnout time = %0.0fs', t(ThrustCounter))
fprintf('\nOff the rail speed: %0.0fft/s', OTRS)
fprintf('\nDescent velocity: %0.0fft/s', v(end-1))
fprintf('\nDrift distance: %0.0fmi', x(step)/5280)

%% Altitude vs. Time
figure
plot(t,h,'LineWidth',1.5);
ap = gca;
ap.Box = 'on'; 
ap.LineWidth = 1.5;

%% altitude and velocity comparisons
figure
yyaxis right
plot(t,h,'LineWidth',1.5);
ylabel('ALTITUDE (ft)')
ylim([0 10^6])
yyaxis left
plot(t,v,'LineWidth',1.5);
ylabel('VELOCITY (ft / sec)')
xlabel('TIME (seconds)')
ylim([0 10^4])
ap = gca;
ap.Box = 'on'; 
ap.LineWidth = 1.5;

%% acceleration time plot
figure
plot(t,ay./32.2,'LineWidth',1.5)
xlim([0 55]) 
ylim([0 15])
xlabel('TIME (seconds)')
ylabel('ACCERLATION (g''s)')
ap = gca;
ap.Box = 'on'; 
ap.LineWidth = 1.5;
set(gca,'XTick',0:2:52);
set(gca,'YTick',0:1:15);

%% altitude vs. drift plot

%Main
figure
hold on
plot(x,h,'LineWidth',1.5)
xlabel('DRIFT (ft)')
ylabel('ALTITUDE (ft)')
ax = gca;
ax.Box = 'on'; 
ax.LineWidth = 1.5;
xlim([0 3*10^5])
ylim([0 8*10^5])
set(gca,'XTick',0:0.5*10^5:2.5*10^5);
set(gca,'YTick',0:10^5:8*10^5);

%Labels
plot(x(ThrustCounter),h(ThrustCounter),'s','LineWidth',1.5)
plot(x(ApogeeCounter),h(ApogeeCounter),'s','LineWidth',1.5)
plot(x(end-1),h(end-1),'s','LineWidth',1.5)
text(x(ThrustCounter),h(ThrustCounter),{'    Burnout',...
    '    t = 45.8s','    H=104700ft','    V=6448ft/s'});
text(x(ApogeeCounter),6.5*10^5,{'    Apogee',...
    '    t = 251s','    H=744330ft','    V=541ft/s'},...
    'HorizontalAlignment','center');
text(x(end-1),10^5,{' Impact',...
    '    t = 493s','    R=48mi'},...
    'HorizontalAlignment','center');
