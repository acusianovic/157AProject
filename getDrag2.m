function Cd = getDrag2(rocket,h,v,a)

%% Rocket's dimensions
d = rocket.geo.body.D; %[in]
L = rocket.data.length.L; %total length [in]
Sb = pi*(d)*L; %body surface area [in^2]
Cr = rocket.geo.fin.c*12; %fin root chord [in]
lambda = rocket.geo.fin.TR;
tc = rocket.geo.fin.ThR; %nondimensional fin thickness []
nf = rocket.geo.fin.n;%number of fins
Sf = rocket.geo.fin.S*144; %fin surface area [in^2]
Lp = 0; %launch lug length [in]
Lap = 0; %nose to launch lug length [in]
Ap = 0; %maximum cross section area of launch lug [in^2]
Sp = 3; %wetted surface area of proturbance [in^2]
Ln = rocket.data.length.nosecone; %nose length [in]
L0 = L - Ln; %body length [in]
xTc = 0.3*Cr; %Location of maximum thickness of wing/chord
db = d; %[in]
Sn = rocket.geo.nc.S*144;% area of the nosecone [in^2]

% Incompressible Skin Friction Coeff. w/ roughness
K = 0.00008;        % polished metal/wood
%K = 0.00025;        % smooth matte paint, carefully  applied

%% Speed of Sound ft/s

% if h < 37000           % Less than 37,000 ft
%     a = -0.004*h + 1116.45;
% elseif h <= 64000
%     a = 968.08;
% else
%     a = 0.0007*h + 924.99;
% end
if abs(v) < 20
    Cd = 0.3;
    return;
elseif v < 0
    v = abs(v);
end
% % Mach Number
M = abs(v/a);

%% Kinematic Viscosity ft^2/s

if h < 15000
    nu = 0.000157*exp(2.503e-5*h);
elseif h <= 30000
    nu = 0.000157*exp(2.76e-5*h - 0.03417);
elseif h <= 248160
    nu = 0.000157*exp(4.664e-5*h - 0.6882);
else
    nu = 5;
end


%% Body's Friction Drag 
CdB = bodyFrictionDrag(M,L,d,Sb,K,a,nu);

%% Fins'Friction Drag
CdF = finFrictionDrag(M,Cr,nu,K,xTc,tc,nf,Sf,d,lambda,a);

%% Interference Drag
CdP = interferenceDrag(M,Lp,nu,K,Lap,Ap,d,Sp);

%% Drag due to rivets, joints,....
Sr = Sb + Sf + Sp + Sn;              % Total Wetted Area of Rocket
Cde = excrescenciesDrag(M,Sr,d);

%% Total Skin Friction Drag
Kf = 1.04;          % Interference Factor
CdFriction = CdB + Kf*CdF + Kf*CdP + Cde;

%% Base Drag Coeff.
Kb = 0.0274*atan(L0/d + 0.0116);
n = 3.6542*(L0/d)^-0.2733;
% get base drag at Mach 0.6
CdB06 = bodyFrictionDrag(0.6,L,d,Sb,K,a,nu);
CdF06 = finFrictionDrag(0.6,Cr,nu,K,xTc,tc,nf,Sf,d,lambda,a);
CdP06 = interferenceDrag(0.6,Lp,nu,K,Lap,Ap,d,Sp);
Cde06 = excrescenciesDrag(0.6,Sr,d);
CdFriction06 = CdB06 + Kf*CdF06 + Kf*CdP06 + Cde06;
CdBase06 = Kb*(db/d)^n/sqrt(CdFriction06);

if M <= 0.6
    CdBase = Kb*(db/d)^n/sqrt(CdFriction);
elseif M < 1
    fb = 1 + 215.8*(M - 0.6)^6;
    CdBase = CdBase06 * fb;
elseif M <= 2
    fb = 2.0881*(M - 1)^3 - 3.7938*(M - 1)^2 ...
        +1.4618*(M - 1) + 1.883917;
    CdBase = CdBase06 * fb;
elseif M <= 4
    fb = 0.297*(M - 2)^3 - 0.7937*(M -2)^2 ...
        - 0.1115*(M - 2) + 1.64006;
    CdBase = CdBase06 * fb;
else
    fb = 0.6;
    CdBase = CdBase06 * fb;
end


%% Transonic Drag

% Transonic Divergence Mach Number
Md = -0.0156*(Ln/d)^2 + 0.136*(Ln/d) + 0.6817;

% Rocket Effective Length       Le = L if nose doesn't bulge
%       Le =  length from nose to end of bulging section
Le = L;                          

% Mach Number Transonic
if Ln/Le < 0.2
    aCoeff = 2.4;
    bCoeff = -1.05;
    Mf = aCoeff*(Le/d)^bCoeff+1.0175;
else
    aCoeff = -321.94*(Ln/Le)^2 + 264.07*(Ln/Le) - 36.348;
    bCoeff = 19.634*(Ln/Le)^2 - 18.369*(Ln/Le) + 1.7434;
    Mf = aCoeff*(Le/d)^bCoeff+1.0175;
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

%% Supersonic Drag
if M >= Mf
    delCdS = delCdMax;
else
    delCdS = 0;
end

%% Total Drag Coeff.
CdF_cor = CdF*Kf;
CdP_cor = CdP*Kf;

Cd = real(CdB + CdF_cor + CdP_cor + Cde + CdBase + delCdT + delCdS);

% if ~isreal(Cd)
%     warning("Imaginary Drag Coefficient!")
%     Cd = 0.3;
% end

%%%Overwrite instability at high Mach
% if M >= 3.5
%    Cd = 0.2; 
% end

end


%% Supporting Functions
function CdB = bodyFrictionDrag(M,L,d,Sb,K,a,nu)
% Compressive Reynolds Number
RnBody = (a.*M.*L)./(12.*nu).*(1 + 0.0283.*M - 0.043*M.^2 + 0.2107*M.^3 ...
    - 0.03829*M.^4 + 0.002709*M.^5);

% Incompressible Skin Friction Coeff.
CfStarBody = 0.037036*RnBody.^(-0.155079);

% Compressible Skin Friction Coeff.
CfBody = CfStarBody*(1 + 0.00798*M - 0.1813*M.^2 + 0.0632*M.^3 ...
    - 0.00933*M.^4 + 0.000549*M.^5);

CfStarRBody = (1.89 + 1.62*log10(L/K))^(-2.5);

% Compressible Skin Friction Coeff. w/ roughness
CfRBody = CfStarRBody/(1 + 0.2044.*M.^2);

% Final Skin Friction Coeff.
if CfBody >= CfRBody
    CfFinalB = CfBody;
else
    CfFinalB = CfRBody;
end

% Body Drag Coeff. due to friction
CdB = CfFinalB*(1 + 60/(L/d)^3 + 0.0025*(L/d))*4*Sb...
    /(pi*d^2);
end

%% Fin Friction Drag
function CdF = finFrictionDrag(M,Cr,nu,K,xTc,tc,nf,Sf,d,lambda,a)
% Compressible Reynolds Number
RnFin = (a.*M.*Cr)./(12*nu).*(1 + 0.0283*M - 0.043*M.^2 ...
    + 0.2107*M.^3 - 0.03829*M.^4 + 0.002709*M.^5);

% Incompressible Skin Friction Coeff.
CfStarFin = 0.037036*RnFin.^(-0.155079);

% Compressible Skin Friction Coeff.
CfFin = CfStarFin*(1 + 0.00798*M - 0.1813*M.^2 + 0.0632*M.^3 ...
    - 0.00933*M.^4 + 0.000549*M.^5);

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
Re = (a.*M.*Cr)/(12*nu);

% Average Flat Plate Skin Friction Coeff. for each fin

if lambda == 0
   CfLambda = CfFinalF.*(1+ 0.5646./log10(Re)); 
else
    CfLambda = CfFinalF.*(log10(Re)).^(2.6)/(lambda^2 - 1) ...
        .*(lambda^2./(log10(Re.*lambda)).^(2.6) - (log10(Re)).^(-2.6) ...
        + 0.5646*lambda^2./(log10(Re.*lambda)).^(3.6) ...
        -0.5646.*(log10(Re)).^(-3.6));
end

% Drag Coeff. for all fins
xBar = xTc/Cr;      % nondimensional location of maximum thickness   

% Drag Coeff. of all fins
CdF = CfLambda.*(1 + 60*tc^4 + 0.8*(1 + 5*xBar^2)*tc)...
    *4*nf*Sf/(pi*d^2);
end

function [CdP] = interferenceDrag(M,Lp,nu,K,Lap,Ap,d,Sp)

if Lp ~= 0
    % Compressible Reynolds Number
    RnP = (a.*M.*Lp)/(12*nu).*(1 + 0.0283*M - 0.043*M.^2 + 0.2107*M.^3 ...
        - 0.03829*M.^4 + 0.002709*M.^5);
    % Incompressible Skin Friction Coeff.
    CfStarP = 0.037036*RnP.^(-0.155079);
    % Compressible Skin Friction Coeff.
    CfP = CfStarP*(1 + 0.00798*M - 0.1813*M.^2 + 0.0632*M.^3 ...
        - 0.00933*M.^4 + 0.000549*M.^5);
    % Incompressible Skin Friction Coeff. w/ roughness
    CfStarRP = (1.89 + 1.62*log10(Lp/K))^-2.5;
    % Compressible Skin Friction Coeff. w/ roughness
    CfRP = CfStarRP/(1 + 0.2044*M.^2);
    % Final Skin Friction Coeff.
    if CfP >= CfRP
        CfFinalP = CfP;
    else
        CfFinalP = CfRP;
    end
    % Friction Coeff. of Protuberance
    CfFinalID = 0.815*CfFinalP.*(Lap/Lp)^-0.1243;
    % Wetted Area of Protuberance, Sp
    % Drag Coeff. of Protuberance due to Friction
    CdP = CfFinalID*(1 + 1.798*(sqrt(Ap)/Lp)^1.5)*4*Sp/(pi*d^2);
else
    CdP = 0;
end

end

function Cde = excrescenciesDrag(M,Sr,d)
if M < 0.78
    Ke = 0.00038;
    Cde = Ke*4*Sr/(pi*d^2);
elseif M <= 1.04
    Ke = -0.4501*M^4 + 1.5954*M^3 - 2.1062*M^2 ...
        + 1.2288*M - 0.26717;
    Cde = Ke*4*Sr/(pi*d^2);
elseif M <= 3
    Ke = 0.0002*M^2 - 0.0012*M + 0.0018;
    Cde = Ke*4*Sr/(pi*d^2);
else
    Ke = 0;
    Cde = 0;
end
end