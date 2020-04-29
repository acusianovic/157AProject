function rocket = Get2DTraj(rocket)
%2D flight trajectory simulator
%   This function calculates the altitude and horizontal displacement of
%   the rocket over time.  It is currently set to terminate at the apogee.
%   This is changed by changing the condition in the while loop to h(step)
%   >= 0 rather than vy(step) >= 0.  Outputs: apogee, OTRS

load('Aerobee150ADragData.mat','Aerobee150ADragData');

%%% Physical Parameters %%%
g0 = 32.174; %sea level gravitational acceleration [ft/s^2]
Pa = 14.7; %sea level atmsopheric pressure [psia]

%%% Define and Re-Name Rocket Geometry %%%
RocketDiam = rocket.geo.body.D/12; %body diameter [ft]
L = rocket.geo.body.L; %length of body [in]
Sb = pi*RocketDiam*12*L; %body surface area [in^2]
nf = rocket.geo.fin.n; %number of fins
Sf = rocket.geo.fin.S_wet/144; %fin surface area [in^2]
Ln = rocket.geo.nc.L; %length of the nosecone [in]
L0 = L - Ln; %body length [in]
db = RocketDiam*12; %rocket diameter in inches 
xTc = rocket.geo.fin.h_t; %distance from fin leading edge to maximum thickness
Cr = rocket.geo.fin.c; %fin root chord [in]
Ct = rocket.geo.fin.c*rocket.geo.fin.TR; %fin tip chord [in]
tc = rocket.geo.fin.ThR*rocket.geo.fin.c; %fin thickness [in]
Sp = 0; %launch lug wetted area [in^2]
Lp = 0; %launch lug length [in]
Lap = 0; %nose tip to launch lug distance [in]
Ap = 0; %maximum cross section area of launch lug [in^2]

%%% Define Propulsion Details %%%
Isp = rocket.prop.Isp; % [s]
NozzleExitArea = rocket.prop.Ae; %[in^2]
mdot = rocket.prop.mdot/32.2; %[slugs/s] 
LaunchAngle = 4*pi/180; %[rad]
AOA = LaunchAngle; %angle of attack[rad] %%%%UPDATE LATER!!!
Pe = rocket.prop.P_e; %assume perfectly expanded at sea level!

%%% Define rocket masses %%%
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

%%% Recovery Altitude %%%
RecoveryAltitude = 300000; %[ft]

%%% Loop Parameters %%%
dt = 0.1; %time step [s]
step = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

%counters
ThrustCounter = 0; %for finding burnout parameters later
ChuteDeployed = 0;

while vy(step) >= 0 && step <= MaxIterations %currently only calculating up to apogee
    
    %%% Forces %%%
    
    %Thust
    if m(step) > DryMass %during burn
        %update ambient pressure
        [~, ~, Pa, rho] = atmos(h(step)/3.28);
        Pa = Pa/101325*14.7; % psi
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
    %get local air density
    rhoAir = rho/515.379; % [slugs/ft^3]
    %get speed of sound
    if h(step) < 37000           
        SOS = -0.004*h(step) + 1116.45;
    elseif h(step) <= 64000
        SOS = 968.08;
    else
        SOS = 0.0007*h(step) + 924.99;
    end
    %dynamic drag model
    if h(step) <= 1000 && ChuteDeployed == 0 %pre-launch and early region
        Af = (pi/4)*RocketDiam^2;
        %Cd(step) = 0.1;
        Cd(step) = interp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),v(step)/SOS);
        Mach(step) = v(step)/1116.28;
        Sign = -1;
    elseif vy(step) > 0 %before apogee
        %[Cd(step),Mach(step)] = Drag(h(step),L,Ct,Cr,xTc,tc,nf,Sp,Lap,Ap,db,L0,Ln,RocketDiam*12,v(step),Sb,Sf,Lp);
        Cd(step) = interp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),v(step)/SOS);
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = -1;
    elseif vy(step) < 0 && h(step) > RecoveryAltitude && ChuteDeployed == 0 %after apogee, before chute deployment
        %[Cd(step),Mach(step)] = Drag(h(step),L,Ct,Cr,xTc,tc,nf,Sp,Lap,Ap,db,L0,Ln,RocketDiam*12,v(step),Sb,Sf,Lp);
        Cd(step) = interp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),v(step)/SOS);
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        Sign = 1;
    else %thereafter: chute out, change sign depending on direction until balance
        if ChuteDeployed == 0
        ChuteDeployed = 1;
        fprintf( 'Main parachute deployed at %f s and %f ft', t(step), h(step))
        end
        Af = (pi/4)*24^2; %[ft^2]
        %Cd(step) = Drag(h(step),L,Ct,Cr,xTc,tc,nf,Sp,Lap,Ap,db,L0,Ln,RocketDiam*12,v(step),Sb,Sf,Lp);
        Cd(step) = interp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),v(step)/SOS);
        AOA = 0; %manually adjust angle of attack !!!!!!!!
        Sign = (-vy(step)/abs(vy(step)));
    end
    Fd(step) = Sign*0.5*rhoAir*v(step)^2*Af*Cd(step);
    %{
    %override unsteady densities
    if h(step) >= 150000
       Cd(step) = 0;
       Fd(step) = 0;
    end
    %}
    
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
    LRL = 160;
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

%%% Outputs %%%
rocket.data.performance.OTRS = OTRS; %[ft/s]
rocket.data.performance.apogee = h(step); %[ft]
rocket.data.performance.range = x(step); %[ft]

end

