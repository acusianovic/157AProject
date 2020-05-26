function [xDif,xCP,CNT,CNFB,XF,CNN,xN] = Barrowman( rocket, AOA )
%Barrowman Stability Analysis
%   Input the rocket and AOA

CNN = 2; %normal force on nose cone [rad^-1]
Alpha = 0.333; %elliptical nose cone

xCG = rocket.data.CG.wet; %from tip of nose cone [in]

%{
LN = rocket.data.length.nosecone; %nose cone length [in]
D = rocket.geo.body.D; %body tube diamter [in]

Fins = rocket.geo.fin.n; %number of fins
CR = rocket.geo.fin.c*12; %fin root-chord [in]
CT = CR*rocket.geo.fin.TR; %fin tip-chord [in]
XF = rocket.geo.fin.LE + rocket.geo.nc.L*12;%distance from nose tip to fin-root chord leading edge [in]
S = rocket.geo.fin.b*12; %Fin semi-span [in]
XS = S*tand(rocket.geo.fin.sweep); %distance between fin root leading edge and fin tip leading edge parallel to body [in]
xN = Alpha*LN; %COP of nose, from tip of nose cone [in]
Kfb = 1+D/(2*S+D); %fin/body interference factor
LF = sqrt(S^2+(XS+CT/2-CR/2)^2); %span length at chord of fins [in]

if Fins == 4
    Beta = 16; %[rad^-1]
elseif Fins == 3
    Beta = 13.85; %[rad^-1]
else
    error('Number of fins must be 3 or 4')
end
CNF = Beta*(S/D)^2/(1+sqrt(1+(2*LF/(CR+CT))^2)); %normal force on N fins [rad^-1]
CNFB = Kfb*CNF; %normal force on fins in presence of the body [rad^-1]
xF = XF+XS*(CR+2*CT)/(3*(CR+CT))+(1/6)*(CR+CT-CR*CT/(CR+CT)); %center of pressure location (fins) [in]
Lb = XF-LN+CR;
xB = LN+Lb/2; %CP body
CNB = (4/pi)*(Lb/D)*AOA; %normal force on body [rad^-1]
CNT = CNN+CNFB+CNB; %total normal force
xCP = (CNN*xN+CNFB*xF+CNB*xB)/CNT; %center of pressure
xDif = (xCP-xCG)/12; %[ft]
%}
%%

d = rocket.geo.body.D/12;          % Rocket diameter, ft
CnaN = 2.0;                        % /rad
Ln = rocket.geo.nc.L;           % Nose Cone length, ft
xN = 0.333 * Ln;
nf = rocket.geo.fin.n;          % Number of fins
if nf == 3
    beta = 13.85;
elseif nf == 4
    beta = 16;
end
b = rocket.geo.fin.b;     % Semipan of fin, ft
Cr = rocket.geo.fin.c;      % Root Chord, ft
Ct = rocket.geo.fin.TR*Cr;  % Tip Chord, ft
XS = b*tand(rocket.geo.fin.sweep);  % Sweep length, ft
XF = (rocket.geo.fin.LE/12 + rocket.geo.nc.L);% Fin location, ft
Kfb = 1 + d/(2*b + d);
LF = sqrt(b^2 + (XS + (Ct/2) - (Cr/2))^2);
CnaF = beta*(b/d)^2 /(1 + sqrt( 1 + (2*LF/(Cr + Ct))^2 ) );
CnaFB = Kfb * CnaF;
xF = XF + (XS/3)*(Cr + 2*Ct)/(Cr + Ct) +...
    (1/6)*(Cr + Ct - (Cr*Ct)/(Cr + Ct));
Lb = (XF - Ln) + Cr;       
CnaB = (4/pi) .* (Lb/d).* AOA;
xB = Ln + (Lb/2); % ft
CnaT = CnaN + CnaFB + CnaB;
xCP = (CnaN*xN + CnaFB*xF + CnaB*xB)./CnaT;

xCP = xCP*12;
xDif = (xCP-xCG)/12; %ft
CNT = CnaT;
CNFB = CnaFB;
CNN = CnaN;
end