function [Cd] = Drag(h,L,Ct,Cr,xTc,nf,Sp,Lap,Ap,db,L0,Ln)

% h = altitude, L is the total length of the rocket
% xTc = the location of maximum thickness of airfoil
% nf = number of fin
% Sp = Wetted Area of Protuberance(launch plug)
% Lap = the distance from nose to Launch plug
% Ap = the maximum cross section of protuberance
% db = area at the base
% L0 = length of section where the diamter is biggest
% Le =  length from nose to end of bulging section
% Le = L if nose doesn't bulge
% Ln = Nosecone's length


% Calculate Speed of Sound (ft/s)    
if h < 37000           % Less than 37,000 ft
    a = -0.004*h + 1116.45;
elseif h <= 64000
    a = 968.08;
else
    a = 0.0007*h + 924.99;
end

% Calclate kinematic viscosity (ft^2/s)
if h < 15000
    nu = 0.000157*exp(2.503e-5*h);
elseif h <= 30000
    nu = 0.000157*exp(2.76e-5*h - 0.03417);
else
    nu = 0.000157*exp(4.664e-5*h - 0.6882);
end

% Body Friction Drag

Sb = pi * d * L;

% Compressive Reynolds Number
RnBody = (a.*M.*L)./(12*nu) *(1 + 0.0283*M - 0.043*M.^2 + 0.2107*M.^3 ...
    - 0.03829*M.^4 + 0.002709*M.^5);

% Incompressible Skin Friction Coeff.
CfStarBody = 0.037036*RnBody^(-0.155079);

% Compressible Skin Friction Coeff.
CfBody = CfStarBody*(1 + 0.00798*M - 0.1813*M^2 + 0.0632*M^3 ...
    - 0.00933*M^4 + 0.000549*M^5);

% Incompressible Skin Friction Coeff. w/ roughness
K = 0.00025;        % smooth matte paint, carefully  applied
CfStarRBody = (1.89 + 1.62*log10(L/K))^-2.5;

% Compressible Skin Friction Coeff. w/ roughness
CfRBody = CfStarRBody/(1 + 0.2044*M^2);

% Final Skin Friction Coeff.
if CfBody >= CfRBody
    CfFinalB = CfBody;
else
    CfFinalB = CfRBody;
end

% Body Drag Coeff. due to friction
CdBody = CfFinalB*(1 + 60/(L/d)^3 + 0.0025*(L/d))*4*Sb...
    /(pi*d^2);

% Fin

% Compressible Reynolds Number
RnFin = (a*M*Cr)/(12*nu) * (1 + 0.0283*M - 0.043*M^2 ...
    + 0.2107*M^3 - 0.03829*M^4 + 0.002709*M^5);

% Incompressible Skin Friction Coeff.
CfStarFin = 0.037036*RnFin^(-0.155079);

% Compressible Skin Friction Coeff.
CfFin = CfStarFin*(1 + 0.00798*M - 0.1813*M^2 + 0.0632*M^3 ...
    - 0.00933*M^4 + 0.000549*M^5);

% Incompressible Skin Friction Coeff. w/ roughness
CfStarRFin = (1.89 + 1.62*log10(Cr/K))^(-2.5);

% Compressible Skin Friction Coeff. w/ roughness
CfRFin = CfStarRFin/(1 + 0.2044*M.^2);

% Final Skin Friction Coeff.
if CfFin >= CfRFin
    CfFinalF = CfFin;
else
    CfFinalF = CfRFin;
end
    
% Incompressible Reynold Number
Re = (a*M*Cr)/(12*nu);

% Taper Ratio
lambda = Ct/Cr;

% Average Flat Plate Skin Friction Coeff. for each fin

if lambda == 0
   CfLambda = cfFinalF*(1+ 0.5646./log10(Re)); 
else
    CfLambda = CfFinalF*(log10(Re)).^2.6/(lambda^2 - 1) ...
        *(lambda^2/(log10(Re*lambda)).^2.6 - (log10(Re)).^-2.6 ...
        + 0.5646*lambda^2/(log10(Re*lambda)).^3.6 ...
        -0.5646*(log10(RnFin)).^-3.6);
end

% Drag Coeff. for all fins

tc = t/Cr;              % Maximum Thickness @ root/ Root Chord
xBar = xTc/Cr;         
Sf = b/2*(Cr + Ct);     % Wetted Area of each fin

% Drag Coeff. of all fins
CdF = CfLambda.*(1 + 60*tc^4 + 0.8*(1 + 5*xBar^2)*tc)...
    *4*nf*Sf*(pi*d^2);

% Interference Drag

% Compressible Reynolds Number
RnInt = (a.*M.*Lp)/(12*nu) *(1 + 0.0283*M - 0.043*M.^2 + 0.2107*M.^3 ...
    - 0.03829*M.^4 + 0.002709*M.^5);

% Incompressible Skin Friction Coeff.
CfStarInt = 0.037036*RnInt.^(-0.155079);

% Compressible Skin Friction Coeff.
CfInt = CfStarInt*(1 + 0.00798*M - 0.1813*M^2 + 0.0632*M^3 ...
    - 0.00933*M^4 + 0.000549*M^5);

% Incompressible Skin Friction Coeff. w/ roughness
CfStarRInt = (1.89 + 1.62*log10(Lp/K))^-2.5;

% Compressible Skin Friction Coeff. w/ roughness
CfRInt = CfStarRInt/(1 + 0.2044*M^2);

% Final Skin Friction Coeff.
if CfInt >= CfRInt
    CfFinal = CfInt;
else
    CfFinal = CfRInt;
end
    
% Friction Coeff. of Protuberance
CfFinalID = 0.815*CfFinal.*(Lap/Lp)^-0.1243;

% Drag Coeff. of Protuberance due to Friction
CdP = CfFinalID*(1 + 1.798*(sqrt(Ap)/Lp)^1.5)*4*Sp/(pi*d^2);   

% Drag due to rivets, joints,....

Sr = Sb + Sf + Sp;              % Total Wetted Area of Rocket
 
if M < 0.78
    Ke = 0.00038;
    Cde = Ke*4*Sr/(pi*d^2);
elseif M <= 1.04
    Ke = -0.4501*M(ii)^4 + 1.5954*M(ii)^3 - 2.1062*M(ii)^2 ...
        + 1.2288*M - 0.26717;
    Cde = Ke*4*Sr/(pi*d^2);
else
    Ke = 0.0002*M(ii)^2 - 0.0012*M(ii) + 0.0018;
    Cde = Ke*4*Sr/(pi*d^2);
end
    
% Total Skin Friction Drag

Kf = 1.04;          % Interference Factor
CdFriction = CdBody + Kf*CdF + Kf*CdP + Cde;

% Base Drag Coeff.

Kb = 0.0274*atan(L0/d + 0.0116);
n = 3.6542*(L0/d)^-0.2733;
       
if M < 0.6
    CdBase = Kb*(db/d)^n/sqrt(CdFriction);
elseif M <1
    fb = 1 + 215.8*(M - 0.6)^6;
    CdBase = Kb*(db/d)^n/sqrt(CdFriction) + fb;
elseif M <= 2
    fb = 2.0881*(M(ii) - 1)^3 - 3.7938*(M - 1)^2 ...
        +1.4618*(M(ii) - 1) + 1.883917;
    CdBase = Kb*(db/d)^n/sqrt(CdFriction) + fb;
else
    fb = 0.297*(M(ii) - 2)^3 - 0.7937*(M -2)^2 ...
        - 0.1115*(M(ii) - 2) + 1.64006;
    CdBase = Kb*(db/d)^n/sqrt(CdFriction) + fb;
end
    
% Transonic Drag

% Transonic Divergence Mach Number
Md = 0.0156*(Ln/d)^2 + 0.136*(Ln/d) + 0.6817;                        

% Mach Number Transonic
if Ln/Le < 0.2
    aCoeff = 2.4;
    bCoeff = -1.05;
    Mf = aCoeff(Le/d)^bCoeff;
else
    aCoeff = -321.94*(Ln/Le)^2 + 264.07*(Ln/Le) - 36.348;
    bCoeff = 19.634*(Ln/Le)^2 - 19.369*(Ln/Le) + 1.7434;
    Mf = aCoeff(Le/d)^bCoeff;
end
   
% Max Rise in Drag
c = 50.676*(Ln/L)^2 - 51.734*(Ln/L) + 15.642;
g = -2.2538*(Ln/L)^2 + 1.3108*(Ln/L) - 1.7344;

if Le/d >= 6
    delCdMax = c*(Le/d)^g;
else
    delCdMax = c*(6)^g;
end

% Drag drise for a given M
if (Md <= M) && (M <= Mf)
    x = (M - Md)/(Mf - Md);
    F = -8.347*x^5 + 24.543*x^4 - 24.946*x^3 + 8.6321*x^2 ...
        + 1.1195*x;
    delCdT = delCdMax*F;
else
    delCdT = 0;
end

% Supersonic Drag
if M >= Mf
    delCdS = delCdMax;
else
    delCdS = 0;
end

% Total Drag Coeff.

Cd = CdBody + Kf*CdF + Kf*CdP +Cde + CdBase + delCdT + delCdS; 

end

