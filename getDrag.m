function [Cd,CdB,CdF_cor,CdP_cor,Cde,CdBase,delCdT,delCdS] = getDrag(rocket,h,v)

d = rocket.geo.body.D; % rocket diameter [in]
L = rocket.data.length.L; % total length of rocket [in]
lambda = rocket.geo.fin.TR;
c = rocket.geo.fin.c; % chord length, in
xTc = rocket.geo.fin.h_t*c; % location of maximum thickness [in]
Sb = pi*d*L; %body surface area [in^2]
nf = rocket.geo.fin.n; %number of fins
Sf = rocket.geo.fin.S_wet*144; %fin surface area [in^2]
Ln = rocket.geo.nc.L; %length of the nosecone [in]
L0 = L - Ln; %body length [in]
tc = rocket.geo.fin.ThR*rocket.geo.fin.c; %fin thickness [in]
Sp = 0; %launch lug wetted area [in^2]
Lp = 0; %launch lug length [in]
Lap = 0; %nose tip to launch lug distance [in]
Ap = 0; %maximum cross section area of launch lug [in^2]
db = d; % diameter at base (equal to body diameter for no boatail)

% h = altitude [ft]
% L is the total length of the rocket [in]
% xTc = the location of maximum thickness of airfoil [in]
% tc = maximum thickness of the fins (9% for NACA 0009) [in]
% nf = number of fin
% Sp = Wetted Area of Protuberance(launch plug) [in^2]
% Lap = the distance from nose to Launch plug [in]
% Ap = the maximum cross section of protuberance [in^2]
% db = diameter at the base [in]
% L0 = length of section where the diamter is biggest [in]
% Le =  length from nose to end of bulging section [in]
% Le = L if nose doesn't bulge [in]
% Ln = Nosecone's length [in]
% d = rocket diameter [in^2]
% v = velocity [ft/s]
% Sb = total wetted area [in^2]
% Sf = wetted fin area [in^2]
% Lp = length of proturbance

Le = L;
%%% Calculate the speed of sound (ft/s)
if h < 37000           
    a = -0.004*h + 1116.45;
elseif h <= 64000
    a = 968.08;
else
    a = 0.0007*h + 924.99;
end

%%% Kinematic Viscosity (ft^2/s)
if h < 15000
    nu = 0.000157*exp(2.503e-5*h);
elseif h <= 30000
    nu = 0.000157*exp(2.76e-5*h - 0.03417);
else
    nu = 0.000157*exp(4.664e-5*h - 0.6882);
end

%%% Mach number

M = v/a;

%%% Body's Friction Drag 

% Compressive Reynolds Number
RnBody = (a.*M.*L)./(12.*nu).*(1 + 0.0283.*M - 0.043*M.^2 + 0.2107*M.^3 ...
    - 0.03829*M.^4 + 0.002709*M.^5);

% Incompressible Skin Friction Coeff.
CfStarBody = 0.037036*RnBody.^(-0.155079);

% Compressible Skin Friction Coeff.
CfBody = CfStarBody*(1 + 0.00798*M - 0.1813*M.^2 + 0.0632*M.^3 ...
    - 0.00933*M.^4 + 0.000549*M.^5);

% Incompressible Skin Friction Coeff. w/ roughness
K = 0.00025;        % smooth matte paint, carefully  applied, in
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



%%% Fins'Friction Drag

% Compressible Reynolds Number
RnFin = (a.*M.*c)./(12*nu).*(1 + 0.0283*M - 0.043*M.^2 ...
    + 0.2107*M.^3 - 0.03829*M.^4 + 0.002709*M.^5);

% Incompressible Skin Friction Coeff.
CfStarFin = 0.037036*RnFin.^(-0.155079);

% Compressible Skin Friction Coeff.
CfFin = CfStarFin*(1 + 0.00798*M - 0.1813*M.^2 + 0.0632*M.^3 ...
    - 0.00933*M.^4 + 0.000549*M.^5);

% Incompressible Skin Friction Coeff. w/ roughness
CfStarRFin = (1.89 + 1.62*log10(c/K))^(-2.5);

% Compressible Skin Friction Coeff. w/ roughness
CfRFin = CfStarRFin/(1 + 0.2044*M.^2);

% Final Skin Friction Coeff.
if CfFin >= CfRFin
    CfFinalF = CfFin;
else
    CfFinalF = CfRFin;
end

% Incompressible Reynold Number
Re = (a.*M.*c)./(12*nu);


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
xBar = xTc/c;         
%Sf = b/2*(Cr + Ct);     % Wetted Area of each fin

% Drag Coeff. of all fins
CdF = CfLambda.*(1 + 60*(tc/c)^4 + 0.8*(1 + 5*xBar^2)*(tc/c))...
    *4*nf*Sf/(pi*d^2);



%%% Interference Drag

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
    
    % Drag Coeff. of Protuberance due to Friction
    CdP = CfFinalID*(1 + 1.798*(sqrt(Ap)/Lp)^1.5)*4*Sp/(pi*d^2);
else
    
    CdP = 0;
    Sp = 0;
end


%%% Drag due to rivets, joints,....

Sr = Sb + Sf + Sp;              % Total Wetted Area of Rocket

if M < 0.78
    Ke = 0.00038;
    Cde = Ke*4*Sr/(pi*d^2);
elseif M <= 1.04
    Ke = -0.4501*M^4 + 1.5954*M^3 - 2.1062*M^2 ...
        + 1.2288*M - 0.26717;
    Cde = Ke*4*Sr/(pi*d^2);
else
    Ke = 0.0002*M^2 - 0.0012*M + 0.0018;
    Cde = Ke*4*Sr/(pi*d^2);
end


%%% Total Skin Friction Drag

Kf = 1.04;          % Interference Factor
CdFriction = CdB + Kf*CdF + Kf*CdP + Cde;


%%% Base Drag Coeff.
Kb = 0.0274*atan(L0/d + 0.0116);
n = 3.6542*(L0/d)^-0.2733;
       
if M <= 0.6
    CdBase = Kb*(db/d)^n/sqrt(CdFriction);
elseif M < 1
    fb = 1 + 215.8*(M - 0.6)^6;
    CdBase = Kb*(db/d)^n/sqrt(CdFriction) * fb;
elseif M <= 2
    fb = 2.0881*(M - 1)^3 - 3.7938*(M - 1)^2 ...
        +1.4618*(M - 1) + 1.883917;
    CdBase = Kb*(db/d)^n/sqrt(CdFriction) * fb;
else
    fb = 0.297*(M - 2)^3 - 0.7937*(M -2)^2 ...
        - 0.1115*(M - 2) + 1.64006;
    CdBase = Kb*(db/d)^n/sqrt(CdFriction) * fb;
end


%%% Transonic Drag

% Transonic Divergence Mach Number
Md = -0.0156*(Ln/d)^2 + 0.136*(Ln/d) + 0.6817;                        

% Mach Number Transonic
if Ln/Le < 0.2
    aCoeff = 2.4;
    bCoeff = -1.05;
    Mf = aCoeff*(Le/d)^bCoeff;
else
    aCoeff = -321.94*(Ln/Le)^2 + 264.07*(Ln/Le) - 36.348;
    bCoeff = 19.634*(Ln/Le)^2 - 18.369*(Ln/Le) + 1.7434;
    Mf = aCoeff*(Le/d)^bCoeff;
end

% Max Rise in Drag
cm = 50.676*(Ln/L)^2 - 51.734*(Ln/L) + 15.642;
g = -2.2538*(Ln/L)^2 + 1.3108*(Ln/L) - 1.7344;

if Le/d >= 6
    delCdMax = cm*(Le/d)^g;
else
    delCdMax = cm*(6)^g;
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


%%% Supersonic Drag
if M >= Mf
    delCdS = delCdMax;
else
    delCdS = 0;
end


%%% Total Drag Coeff.
CdF_cor = CdF*Kf;
CdP_cor = CdP*Kf;
Cd = CdB + CdF_cor + CdP_cor + Cde + CdBase + delCdT + delCdS; 

% if M <= 0.8
%     Cd = 0.325;
% elseif M >= 3.5
%     Cd = 0.235;
% end

end

