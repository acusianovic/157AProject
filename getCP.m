function [xCP] = getCP(Shape,Ln,d,tnN,S,XS,Cr,Ct,XF,nf,AoA)

% Shape =  shape of nose cone. 1 for von Karman and 2 for Parabolic
%tnN = nose cone thickness
% nf = number of fins
% XF = distance from Nose to LE of root chord
% S = semi-span
% XS = Chord's LE to Tip's LE
% v = velocity

%%% Nose Cone

if Shape == 1
    % Von Karman Nose Cone
    x = 0:0.1:Ln;
    theta = acos( 1 - (2*x)/Ln);
    R = d/2;
    yN = (R/sqrt(pi)) * sqrt(theta - sin(2.*theta)/2);
    
    yInnerN = yN - tnN;
    
    for ii = 1:length(yN)
        if yInnerN(ii) < 0
            yInnerN(ii) = 0;
        end
    end
    
    % Derivative of yN
    dyN = zeros(1,length(yN));
    dyN(1) = (yN(2) - yN(1))/dx;
    dyN(length(yN)) = (yN(length(yN)) - yN(length(yN)-1))/dx;
    for ii = 2:length(yN)-1
        dyN(ii) = ( yN(ii+1) - yN(ii) )/ dx;
    end
    
    % Surface Area of Nose Cone
    SANose = 2*pi*trapz(x,yN.*sqrt(1 + dyN.^2));
    
    % Nose Cone Area at its base
    ANose = (pi/4) * d^2;
    
    % Rotating curves around x axis
    VOuter = pi*trapz(x,yN);
    VInner = pi*trapz(x,yInnerN);
    
    VNose = VOuter - VInner;
    
    % Center of Pressure of Nose Cone
    xCpNose = (Ln*ANose - VNose)/ANose;
elseif Shape  == 2
    % Parabolic Nose Cone
    xCpNose = 0.5 * Ln;
end


%%% Fin 

if nf == 3
    beta = 13.85;
elseif nf == 4
    beta = 16;
end

% Fin/BOdy Interference Factor
Kfb = 1 + d/(2*S + d);

% Span length at chord of fins
LF = sqrt(S^2 + (XS + (Ct/2) - (Cr/2))^2);

% Normal force on n fins
CnaF = beta*(S/d)^2 /(1 + sqrt( 1 + (2*LF/(Cr + Ct))^2 ) );

% Normal force on body
CnaFB = Kfb * CnaF;

% Center of Press. of fins
xF = XF + (XS/3)*(Cr + 2*Ct)/(Cr + Ct) +...
    (1/6)*(Cr + Ct - (Cr*Ct)/(Cr + Ct));


%%% Lifting Body

% Body length
Lb = (XF - LN) + Cr;       

% Normal Coeff. of Body
CnaB = (4/pi) * (Lb/d).* AoA;

% Location of Normal Force of Body
xB = LN + (Lb/2);

%%% Total Cp

% Total normal force on rocket
CnaT = CnaN + CnaFB + CnaB;

% Center of Press.
xCP = (CnaN*xN + CnaFB*xF + CnaB*xB)/CnaT;


end

