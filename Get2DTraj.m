clc

load('betsyMK6.mat','betsyMK6')
load('atmo_dat.mat','atmo_dat')
load('WindData.mat','WindData')
Altitude = 0:100000;
Altitude = Altitude*3.28084; %[ft]
Meridian = WindData(:,1)*3.28084;%[ft/s]

rocket = betsyMK6;

%%% Physical Parameters %%%
g0 = 32.174; %sea level gravitational acceleration [ft/s^2]

%%% Define and Re-Name Rocket Geometry %%%
RocketDiam = rocket.geo.body.D/12; %body diameter [ft]
L = rocket.data.length.L/12; %nose cone to fuel exit [ft]
xCG = rocket.data.CG.wet/12; %from tip of nose cone [ft]

%%% Define Propulsion Details %%%
cstar = rocket.prop.cstar*3.28; % characteristic velocity, ft/s
mdot = rocket.prop.mdot/32.2; %[slugs/s] 
LaunchAngle = 4*pi/180; %[rad]
Ft = 0;

%%% Define and convert rocket masses %%%
DryMass = rocket.data.weight.dry/32.2; %[slugs]
WetMass = rocket.data.weight.wet/32.2; %[slugs]
m = WetMass; %wet mass [slugs]

%%% Initialize Kinematic Parameters %%%
t = 0; %time [s]
h = 0; %altitude [ft]
x = 0; %drift [ft]
vx = 0; %drift velocity [ft/s]
vy = 0; %vertical velocity [ft/s] 
v = sqrt(vx^2+vy^2); %velocity mag [ft/s]
ax = 0; %x acceleration [ft/s^2]
ay =0; %y acceleration [ft/s^2]

%%% Recovery Parameters %%%
RecoveryAltitude = 1000; %[ft]
DrogueArea = (pi/4)*1^2; %[ft^2]
ChuteArea = (pi/4)*3.7393^2; %[ft^2]
CdDrogue = 1.5;
CdChute = 2.2;

%%% Loop Parameters %%%
dt = 0.001; %time step [s]
step = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

%%% Set IL for now %%%
IL = 2040;

%%% Initialize Stability Parameters %%%
Pitch = LaunchAngle; %Pitching Angle [rad]
FPA = LaunchAngle; %Flight Path Angle [rad]
AoA = 0; %Angle of Attack
omega = 0;
alpha = 0;

%counters
ThrustCounter = 0; %for finding burnout parameters later
DrogueDeployed = 0;
ChuteDeployed = 0;

while vy(step) >= 0 && step <= MaxIterations && x(step) < 5280*50
    
    %%% Forces %%%
    
    %Thust
    if m(step) > DryMass %during burn
        %update ambient pressure
        nu = lininterp1(atmo_dat.Z_L,atmo_dat.nu,h(step)); % kinematic viscosity, ft2/s
        a = lininterp1(atmo_dat.Z_L,atmo_dat.c,h(step)); % speed of sound, ft/s
        Pa = lininterp1(atmo_dat.Z,atmo_dat.P,h(step)); % psi
        rhoAir(step) = lininterp1(atmo_dat.Z,atmo_dat.rho,h(step)); % slug/ft3 
        %update thrust
        Ft(step+1) = mdot*cstar*getThrustCoefficient(rocket,Pa);
        %update mass
        m(step+1) = m(step)-mdot*dt; %[slugs]
        %update thrust counter
        ThrustCounter = ThrustCounter+1;
    else %after burn
        rhoAir(step) = lininterp1(atmo_dat.Z,atmo_dat.rho,h(step)); % slug/ft3 
        a = lininterp1(atmo_dat.Z_L,atmo_dat.c,h(step)); % speed of sound, ft/s
        Ft(step+1) = 0;
        m(step+1) = m(step);
    end
    
    %Gravity
    %update gravitational acceleration
    g = g0*((2.0856*10^7)/(2.0856*10^7+h(step)))^2;
    %calculate new force due to gravity
    Fg(step) = -m(step)*g;

    %get mach number
    Mach(step) = v(step)/a;
    SOS(step) = a;
    
    %Drag
    %choose drag parameters

    if vy(step) >= 0
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        sign = -1;
        Cd(step) = getDrag2(rocket,v(step),a,nu);
        Fd(step) = sign*0.5*rhoAir(step)*v(step)^2*Af*Cd(step);
    elseif vy(step) < 0 && h(step) > RecoveryAltitude && ChuteDeployed == 0 %after apogee, before main chute deployment
        if DrogueDeployed == 0
            DrogueDeployed = 1;
            fprintf( '\n*** Drogue deployed at %0.0fs ***', t(step))
            fprintf( '\nCurrent Altitude: %0.0fmi \nCurrent Drift:%0.0fmi'...
                ,h(step)/5280,x(step)/5280)
            fprintf( '\nCurrent Air Density: %0.9f slugs/ft^3', rhoAir(step))
            DrogueLoopInfo = [t(step) x(step) h(step)];
        end
        sign = 1;
        Af = DrogueArea; %[ft^2]
        Cd(step) = CdDrogue;
        Fd(step) = sign*0.5*rhoAir(step)*vy(step)^2*Af*Cd(step);
    else %after apogee, after main chute deployment
        if ChuteDeployed == 0
            ChuteDeployed = 1;
            fprintf( '\n*** Main deployed at %0.0fs ***', t(step))
            fprintf( '\nCurrent Altitude: %0.0fft \nCurrent Drift:%0.0fmi'...
                ,h(step),x(step)/5280)
            fprintf( '\nCurrent Air Density: %0.9f slugs/ft^3', rhoAir(step))
            MainChuteLoopInfo = [t(step) x(step) h(step)];
        end
        sign = 1;
        Af = ChuteArea; %[ft^2]
        Cd(step) = CdChute;
        Fd(step) = sign*0.5*rhoAir(step)*vy(step)^2*Af*Cd(step);
    end
    
    %get wind speed
    if h(step) < 300000
        vwind = lininterp1(Altitude,Meridian,h(step));
        Fwind(step) = 0.5*rhoAir(step)*vwind^2*Cd(step)*Af;
    else
        vwind = 0;
        Fwind(step) = 0;
    end
     
    %%% Kinematics %%%
    
    %calculate net force in each direction
    Fx = (Ft(step)+Fd(step))*sin(Pitch(step))+Fwind(step);
    Fy = (Ft(step)+Fd(step))*cos(Pitch(step))+Fg(step);
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
    if ( h(step+1) <= 0 || v(step+1) <= 0 ) && step < 100 %we haven't lifted off yet
        x(step+1) = 0;
        h(step+1) = 0;
        vy(step+1) = 0;
        vx(step+1) = 0;
    end
      
    %%% Stability %%%
    if sqrt(x(step)^2+h(step)^2) < 160
        Pitch(step+1) = LaunchAngle;
        FPA(step+1) = LaunchAngle;
        AoA(step+1) = 0;
        q(step) = 0.5*rhoAir(step)*v(step)^2;
    elseif vy(step) > 0
        vrw = sqrt( ( vx(step)-vwind )^2 + vy(step)^2 );
        q(step) = 0.5*rhoAir(step)*vrw^2;
        [xDif,CnT,CnFB,xF,CnN,xN,xB,CnB] = Barrowman( rocket, AoA(step) );
        N = q(step)*Af*CnT*AoA(step);
        Mrest(step) = -N*xDif;
        Coeff = (CnFB*(xF-xCG)^2+CnN*(xN-xCG)^2+CnB*(xB-xCG)^2); % [slugs*ft^2]
        Mdamp(step) = -0.5*rhoAir(step)*Af*omega*Coeff;
        alpha = -(-Mrest(step)-Mdamp(step))/IL; %s^-2
        omega = omega+alpha*dt; %s^-1
        Pitch(step+1) = Pitch(step)+omega*dt; %rad
        FPA(step+1) = atan( ( vx(step)-vwind ) / vy(step) );
        AoA(step+1) = Pitch(step+1)-FPA(step+1);
    else
        Pitch(step+1) = 0;
        FPA(step+1) = 0;
        AoA(step+1) = 0;
        q(step) = 0.5*rhoAir(step)*v(step)^2;
    end
    
    
    %%% find OTRS %%%
    if sqrt(x(step)^2+h(step)^2) <= 160
        OTRS = v(step);
        OTRStep = step;
    end
    
    %%% update time %%%
    t(step+1) = t(step)+dt;
    
    %%% update counter %%%
    step = step+1;
    
end

%%% Outputs %%%
rocket.data.performance.OTRS = OTRS; %[ft/s]
rocket.data.performance.apogee = max(h); %[ft]
rocket.data.performance.range = max(x); %[ft]

%%% Store Betsy MK5 Traj %%%
Traj.x = x; %ft
Traj.h = h; %ft
Traj.v = v; %ft/s
Traj.vx = vx; %ft/s
Traj.vy = vy; %ft/s
Traj.ay = ay; %ft/s^2
Traj.q = q; %psf
Traj.t = t; %time
Traj.IL = IL; %moment of inertia about pitch axis [slugs/ft^2]
Traj.rhoAir = rhoAir; %air density [slugs/ft^2]
Traj.OTRS = [OTRS,OTRStep]; %ft/s
Traj.xCG = xCG; %ft
Traj.Pitch = Pitch*180/pi; %[deg]
Traj.AoA = AoA*180/pi; %[deg]
Traj.FPA = FPA*180/pi; %[deg]

%%%printouts
fprintf('\n*** Performance ***')
fprintf('\nApogee: %0.0fft/s', max(h))
fprintf('\nDrift: %0.0fmi',x(end)/5280)
fprintf('\nFinal Descent Velocity: %0.0fft/s',vy(end))
fprintf('\nOff the Rail Speed: %0.0f ft/s',rocket.data.performance.OTRS)
fprintf('\nFinal Mass: %0.0f lbs',m(end)*32.2)
fprintf('\n*** Maximum Velocity and Acceleration ***')
fprintf('\nMaximum Velocity: %0.0f ft/s',max(v))
fprintf('\nMaximum Mach Number: %0.0f', max(Mach))
%% Plots: Drift & Altitude

figure
hold on
plot(x/5280,h/5280,'LineWidth',1.5)
axis([0 35 -10 75])
xlabel('Drift [mi]')
ylabel('Altitude [mi]')
plot(DrogueLoopInfo(2)/5280,DrogueLoopInfo(3)/5280,'rd','LineWidth',1.5)
A = string(round(DrogueLoopInfo(3)/5280));
A = { strcat('Apogee/Drogue: ',A,'mi') };
labelpoints(DrogueLoopInfo(2)/5280,DrogueLoopInfo(3)/5280,A,'N',0.2)
plot(MainChuteLoopInfo(2)/5280,MainChuteLoopInfo(3)/5280,'rd','LineWidth',1.5)
B = string(round(MainChuteLoopInfo(3)));
B = { strcat('Parachute Deployment: ',B,'ft') };
labelpoints(MainChuteLoopInfo(2)/5280,MainChuteLoopInfo(3)/5280,B,'S',0.2)
ax = gca;
ax.Box = 'on';
ax.FontSize = 11;
ax.LineWidth = 1.5;
ax.GridLineStyle = ':';
grid on
xlabel('Horizontal Drift [mi]');
ylabel('Altitude [mi]');

%% Plots: Velocity & Acceleration
figure
plot(t,vy,'LineWidth',1.5)
ax = gca;
ax.Box = 'on';
ax.FontSize = 11;
ax.LineWidth = 1.5;
ax.GridLineStyle = ':';
grid on
xlabel('Time [s]');
ylabel('Vertical Velocity [ft/s]');

figure
plot(t,ay,'LineWidth',1.5)
ax = gca;
ax.Box = 'on';
ax.FontSize = 11;
ax.LineWidth = 1.5;
ax.GridLineStyle = ':';
grid on
xlabel('Time [s]');
ylabel('Vertical Acceleration [ft/s^2]');

%% Plots: Dynamic Pressure
figure
plot(t(1:end-1),q,'LineWidth',1.5)
hold on
[maxQ,index] = max(q);
plot(t(index),maxQ,'rd','LineWidth',1.5);
A = string(round(maxQ));
A = { strcat('Max Q = ',A,'psf') };
labelpoints(t(index),maxQ,A,'E',0.2)
ax = gca;
ax.Box = 'on';
ax.FontSize = 11;
ax.LineWidth = 1.5;
ax.GridLineStyle = ':';
grid on
xlabel('Time [s]');
ylabel('Dynamic Pressure [psf]');

%% Plots: Dynamic Stability Analysis

figure
hold on
plot(t,Traj.Pitch,'LineWidth',1.5)
plot(t,Traj.FPA,'LineWidth',1.5);
plot(t,Traj.AoA,'LineWidth',1.5);
ax = gca;
ax.Box = 'on';
ax.FontSize = 11;
ax.LineWidth = 1.5;
ax.GridLineStyle = ':';
grid on
xlabel('Time [s]');
ylabel('Angle [degrees]');
leg = legend('Pitch','Flight Path Angle','Angle of Attack');
leg.Location = 'northwest';
%% Harmonic Oscillator Stability Model
clear sign

%%% Input Parameters %%%
m = rocket.data.weight.wet/32.2; %rocket wet mass [slugs]
mdot = rocket.prop.mdot/32.2; %mdot [slugs/s]
Diam = rocket.geo.body.D/12; %rocket diameter [ft]
L = rocket.data.length.L/12; %nose cone to fuel exit [ft]
Ar = (pi/4)*Diam^2; %[ft^2]
xCG = Traj.xCG; %from tip of nose cone [ft]
IL = Traj.IL; %[slugs ft^2]
OTRS = Traj.OTRS(1); %off the rail speed
step = Traj.OTRS(2); %step that OTRS happens

%max wind encountered
maxwind = 15; %[mph]
maxwind = maxwind*5830/3600; %[ft/s]

%AOA effective
Pitcheff = atan( ( vx(step)+maxwind )  / vy(step) );

%%% time %%%
t = 0:0.1:20;

%%% Get static stability parameters %%%
[xDif,CnT,CnFB,xF,CnN,xN,~,~] = Barrowman(rocket,Pitcheff); %[ft,rad^-1]
Sref = (pi/4)*Diam^2; %[ft^2]

c1 = (rhoAir(step)/2)*v(step)^2*Af*CnT*xDif; %[slugs ft^2/s^2]
c2 = (rhoAir(step)/2)*v(step)*Af*(CnFB*(xF-xCG)^2+CnN*(xN-xCG)^2); % [slugs*ft^2]

D = c2/(2*IL);
omega = sqrt( c1/IL-c2^2/4/IL^2 );
phi =  atan( Pitcheff*omega/( D*Pitcheff+Omega ) );
A = Pitcheff/sin(phi);

N = 0.5*rhoAir(step)*v(step)^2*CnT*Pitch(step)*pi*Diam^2/4;
Omega = dt*( N*xDif-c1*Pitcheff )/IL; %change in angular vel [rad/s]

%%% Update Pitch %%%
Pitch = A*exp(-t*D).*sin(omega*t+phi);

figure
hold on
plot(t,Pitch*180/pi,'LineWidth',1.5)
ax = gca;
ax.Box = 'on';
ax.FontSize = 11;
ax.LineWidth = 1.5;
ax.GridLineStyle = ':';
grid on
xlabel('Time [s]');
ylabel('Pitch [deg]');
