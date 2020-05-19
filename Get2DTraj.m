function rocket = Get2DTraj(rocket)
%2D flight trajectory simulator
%   This function calculates the altitude and horizontal displacement of
%   the rocket over time.  It is also pre-configured to report dynamic
%   pressure, mach number, speed of sound, etc.  We are currently using
%   the Aerobee 150A drag model

%load('Aerobee150ADragData.mat','Aerobee150ADragData');

%%% Physical Parameters %%%
g0 = 32.174; %sea level gravitational acceleration [ft/s^2]

%%% Define and Re-Name Rocket Geometry %%%
RocketDiam = rocket.geo.body.D/12; %body diameter [ft]

%%% Define Propulsion Details %%%
cstar = rocket.prop.cstar*3.28; % characteristic velocity, ft/s
mdot = rocket.prop.mdot/32.2; %[slugs/s] 
LaunchAngle = 4*pi/180; %[rad]
AOA = LaunchAngle; %angle of attack[rad]
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
DrogueArea = (pi/4)*2^2; %[ft^2]
ChuteArea = (pi/4)*5^2; %[ft^2]
CdDrogue = 0.5;
CdChute = 0.9;

%%% Loop Parameters %%%
dt = 0.001; %time step [s]
step = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

%counters
ThrustCounter = 0; %for finding burnout parameters later
DrogueDeployed = 0;
ChuteDeployed = 0;

while vy(step) >= 0 && step <= MaxIterations
    
    %%% Forces %%%
    
    %Thust
    if m(step) > DryMass %during burn
        %update ambient pressure
        [~, SOS, Pa, rho] = atmos(h(step)/3.28); %get speed of sound, pressure, density
        Pa = Pa/101325*14.7; % psi
        %update thrust
        Ft(step+1) = mdot*cstar*getThrustCoefficient(rocket,Pa);
        %update mass
        m(step+1) = m(step)-mdot*dt; %[slugs]
        %update thrust counter
        ThrustCounter = ThrustCounter+1;
    else %after burn
        [~, SOS, ~, rho] = atmos(h(step)/3.28);
        Ft(step+1) = 0;
        m(step+1) = m(step);
    end
    
    %Gravity
    %update gravitational acceleration
    g = g0*((2.0856*10^7)/(2.0856*10^7+h(step)))^2;
    %calculate new force due to gravity
    Fg(step) = -m(step)*g;

    %Drag
    %local air density
    rhoAir(step) = rho/515.379; % [slugs/ft^3]
    
    %mach number
    %M(step) = v(step)/(SOS*3.28);
    
    %choose drag coefficient
    %{
    if M(step) < 7
        Cd(step) = lininterp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),M(step));
    else
        Cd(step) = min(Aerobee150ADragData(2,:));
    end
    %}
    
    %choose drag area
    if vy(step) == 0 %no liftoff
        Af = 0;
        sign = 0;
        [Cd(step)] = getDrag2( rocket,h(step),v(step),SOS );
    elseif vy(step) > 0 %before apogee
        Af = (pi/4)*RocketDiam^2; %[ft^2]
        sign = -vy(step)/abs(vy(step));
        [Cd(step)] = getDrag2( rocket,h(step),v(step),SOS );
    elseif vy(step) < 0 && h(step) > RecoveryAltitude && ChuteDeployed == 0 %after apogee, before main chute deployment
        if DrogueDeployed == 0
            DrogueDeployed = 1;
            fprintf( '\nDrogue deployed at %f s and %f ft', t(step), h(step))
        end
        Af = DrogueArea; %[ft^2]
        AOA = 0;
        Cd(step) = CdDrogue;
        sign = -vy(step)/abs(vy(step));
    else %after apogee, after main chute deployment
        if ChuteDeployed == 0
            ChuteDeployed = 1;
            fprintf( '\nMain parachute deployed at %f s and %f ft\n', t(step), h(step))
        end
        Af = ChuteArea; %[ft^2]
        sign = -vy(step)/abs(vy(step));
        Cd(step) = CdChute;
    end
    Fd(step) = sign*0.5*rhoAir(step)*v(step)^2*Af*Cd(step);
    q(step) = 0.5*rhoAir(step)*v(step)^2;
    
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
    if ( h(step+1) <= 0 || v(step+1) <= 0 ) && step < 100 %we haven't lifted off yet
        x(step+1) = 0;
        h(step+1) = 0;
        vy(step+1) = 0;
    end
    %%% find OTRS %%%
    if sqrt(x(step)^2+h(step)^2) <= 160
        OTRS = v(step);
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
rocket.data.performance.vMax = max(v);%[ft/s]

end