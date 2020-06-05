function [xDif,CnT,CnFB,xF,CnN,xN,xB,CnB] = Barrowman( rocket, AOA )
%Barrowman Stability Analysis
%   Input the rocket and AOA

CnN = 2; %normal force on nose cone [rad^-1]
Alpha = 0.333; %elliptical nose cone

xCG = rocket.data.CG.wet; %from tip of nose cone [in]

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

xCP = (CnaN*xN + CnaFB*xF + CnaB*xB)./CnaT; %ft
xDif = xCP-xCG/12; %ft
CnT = CnaT; %rad^-1
CnFB = CnaFB; %rad^-1
CnN = CnaN; %rad^-1
CnB = CnaB; %rad^-1
end