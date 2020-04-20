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
t0 = 0; %time [s]
h0 = 0; %altitude [ft]
x0 = 0; %drift [ft]
vx0 = 0; %drift velocity [fts]
vy0 = 0; %vertical velocity [ft/s] 
Ft = mdot*Isp*g0+(Pe-Pa)*NozzleArea;
Fg = (DryMass+FuelMass)*g0;
LaunchAngle = 4*pi/180; [rad]
%}

%%% Aerobee 150 Benchmark Override %%%
FuelMass = 1900; % [lb] 862kg
DryMass = 149; % [lb] 68kg PL
RocketDiam = 1.25; %[ft] 0.38m
Ft = 4046.6; %liftoff thrust[lbf] 18kN
Isp = 198; %[s]
m = DryMass+Fuelmass;
Fg = m*g0;%[lbf]
mdot = Ft/(g0*Isp); %[slugs/s]
NozzleExitArea = 0.296875; %[ft^2]
LaunchAngle = 4*pi/180; %[rad]
AOA = LaunchAngle; %angle of attack[rad]
L = 292; %length [in]
Sb = pi*RocketDiam*L;

Cr = 39; %fin root chord [in]
Ct = 27; %fin tip chord [in]
FinThick = 1.1; %fin thickness [in]
Nf = 4;%number of fins
Sf = (16.1/2)*(Cr+Ct);

Lp = 0.75; %launch lug length [in]
aL = 262; %nose to launch lug length [in]
Apro = Lp*0.375; %maximum cross section area of launch lug [in^2]
Spro = 3*0.75+2*0.375; %wetted surface area of proturbance [in^2]

%%% Loop Parameters %%%
dt = 0.1; %time step [s]
step = 1; %count loop iterations
MaxIterations = 10^6; %force stop condition

%counters
ThrustCounter = 0; %for finding burnout parameters later
SOSMarker = 0; %avoid looping through speed of sound calculation

while x0 >= 0 && step <= MaxIterations
    
    %%% Forces %%%
    
    %Thust
    if m > DryMass %during burn
        %update ambient pressure
        Pa = interp1(PressureAltitude,AtmosphericPressure,x(step));%[slugs/ft^3]
        %update thrust
        Ft = mdot*g0*Isp+(Pe-Pa)*NozzleExitArea; %[lbf]
        %update mass
        m(step+1) = m(step)-mdot*dt; %[slugs]
        %update thrust counter
        ThrustCounter = ThrustCounter+1;
    else %after burn
        Ft = 0;
    end
    
    %Gravitational Force
    %update gravitational acceleration
    g = g0*((2.0856*10^7)/(2.0856*10^7+x(step)))^2;
    %calculate new force due to gravity
    Fg = m*g;
    
    %%Drag Force%%
    
    %get flow parameters
    
    %speed of sound [ft/s]
    if h(step) <= 37000
        SOS = -0.004*h(step)+1116.45;
    elseif SOSMarker == 0 && h(step) > 37000 && h(step) <= 64000
        SOS = 968.08;
        SOSMarker = SOSMarker+1;
    else
        SOS = 0.0007*h(step)+924.99;
    end
    
    %kinematic viscosity [ft^2/s]
    if h(step)<=15000
        KV = 0.000157*exp(0.00002503*h(step));
    elseif h(step)>15000 && h(step)<= 30000
        KV = 0.000157*exp(0.00002760*h(step)-0.03417);
    else
        KV = 0.000157*exp(0.00004664*h(step)-0.6882);
    end
    
    %mach number
    Mn = v(step)/SOS;
    
    %Reynold's number (body)
    Rn = (SOS*Mn*L)/(12*KV)*(1+0.0283*Mn-0.043*Mn^2+0.2107*Mn^3-0.03829*Mn^4+0.002709*Mn^5);
    
    %body drag
    
    %incompressible skin friction coefficient
    Cfstar = 0.037036*Rn^(-0.155079);
    %compressible skin friction coefficient
    Compf = Cfstar*(1+0.00789*Mn-0.1813*Mn^2+0.0632*M^3-0.00933*M^4+0.000549*M^5);
    %Cfstar with roughness
    CfstarTerm = 1/(1.89+1.62*log10(L/0.00025))^2.5;
    %Cf with roughness
    CompfTerm = CfstarTerm/(1+0.2044*Mn^2);
    %choose skin friction coefficient
    if Compf >= CompfTerm
        FricCoef = Compf;
    else
        FricCoef = CompfTerm;
    end
    %body drag due to friction
    BodyDrag = FricCoef*(1+60/(L/RocketDiam)^3+0.0025*(L/RocketDiam))*(4*Sb/pi/RocketDiam^2);
    
    %fin friction
   
    %Reynold's number (fins)
    Rn = (Rn/L)*Cr;
    %incompressible skin friction coefficient
    Cfstar = 0.037036*Rn^(-0.155079);
    %compressible skin friction coefficient
    Compf = Cfstar*(1+0.00789*Mn-0.1813*Mn^2+0.0632*M^3-0.00933*M^4+0.000549*M^5);
    %same coefficients with roughness included
    CfstarTerm = 1/(1.89+1.62*log10(Cr/0.00025))^2.5;
    CompfTerm = CfstarTerm/(1+0.2044*M^2);
    %chooose fin friction coefficient
    if Compf >= CompfTerm
        FricCoef = Compf;
    else
        FricCoef = CompfTerm;
    end
    %Incompressible Reynold's number (fins)
    Rn = a*Mn*Cr/(12*KV);
    %ratio of fin tip chord to root chord
    lambda = Ct/Cr;
    %average flat plate skin friction coefficient for each fin panel
    Cflambda = FricCoef*log10(Rn)^2.6/(lambda^2-1)*(...
        lambda^2/(log10(Rn*lambda))^2.6-1/(log10(Rn))^2.6+...
        0.5646*(lambda^2/(log10(Rn*lambda))^3.6-1/(log10(Rn))^3.6) );
    %calculate fin drag coefficient
    FinDrag = Cflambda*(1+60*(FinThick/Cr)^4+0.8*(1+5*(0)^2)*(FinThick/Cr)...
        *(4*Nf*Sf/pi/RocketDiam^2));
    
    %proturberance friction drag
    
    %Reynold's number
    Rn = (Rn/Cr)*Lp;
    %incompressible skin friction coefficient
    Cfstar = 0.037036*Rn^(-0.155079);
    %compressible skin friction coefficient
    Compf = Cfstar*(1+0.00789*Mn-0.1813*Mn^2+0.0632*M^3-0.00933*M^4+0.000549*M^5);
    %same but with roughness
    CfstarTerm = 1/(1.89+1.62*log10(Lp/0.00025))^2.5;
    CompfTerm = CfstarTerm/(1+0.2044Mn^2);
    %choose friction coefficient
    if Compf >= CompfTerm
        FricCoef = Compf;
    else
        FricCoef = CompfTerm;
    end
    %friction coefficient of proturbance
    ProCoef = 0.815*FricCoef*(aL/Lp)^(-0.1243);
    %drag coefficient of proturbance due to friction
    ProDrag = ProCoef*(1+1.798*(sqrt(APro)/Lp)^(3/2))*(4*Spro/pi/RocketDiam^2);
    
    %drag due to excrescencies
    
    %choose coefficient for excrescencies drag increment
    if Mn < 0.78
        Ke = 0.00038;
    elseif Mn >= 0.78 && Mn <= 1.04
        Ke = -0.4501*Mn^4+1.5954*Mn^3-2.1062*Mn^2+1.2288*Mn-0.26717;
    else
        Ke = 0.0002*Mn^2-0.0012*Mn+0.0018;
    end
    %drag coefficient due to excrescencies
    ExDrag = Ke*4*Sb/pi/RocketDiam^2;
    
    %TOTAL FRICTION DRAG COEFFICIENT
    FricDrag = BodyDrag + 1.04*(FinDrag+ProDrag)+ExDrag;
    
    %base drag coefficient
    
    
    
end


