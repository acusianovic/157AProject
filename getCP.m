function rocket = getCP(rocket)
AoA = 0:0.25:30;                 % / deg

% Need to include AoA
d = rocket.geo.body.D/12;          % Rocket diameter, ft
CnaN = 2.0;                        % /rad

%% Nose Cone
Ln = rocket.geo.nc.L;           % Nose Cone length, ft
Shape = rocket.geo.nc.Shape;    % Nose Cone Shape
V = rocket.geo.nc.V;            % Nose Cone Volume, ft^3

if Shape == 1           % Von Karman
    dx = 0.1;
    x = 0:dx:Ln;
    theta = acos( 1 - (2*x)/Ln);
    R = d/2;
    yN = (R/sqrt(pi)) * sqrt(theta - sin(2.*theta)/2);
 % Derivative of yN
    dyN = zeros(1,length(yN));
    dyN(1) = (yN(2) - yN(1))/dx;
    dyN(length(yN)) = (yN(length(yN)) - yN(length(yN)-1))/dx;
    for ii = 2:length(yN)-1
        dyN(ii) = ( yN(ii+1) - yN(ii) )/ dx;
    end
 % Surface Area of Nose Cone(ft^2)
    SA_nose = 2*pi*trapz(x,yN.*sqrt(1 + dyN.^2));          
 % Nose Cone Area at its base(ft^2)
    A = (pi/4) * d^2;   
 % Center of Pressure of Nose Cone (ft)
    xN = (Ln*A - V)/A;
    
elseif Shape  == 2          % 1/2 Power
    xN = 0.5 * Ln;
    
elseif Shape == 3           % Elliptical
    xN = 0.333 * Ln;
end

%% Fin 
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
XF = rocket.geo.fin.LE/12;            % Fin location, ft

% Fin/BOdy Interference Factor
Kfb = 1 + d/(2*b + d);

% Span length at chord of fins
LF = sqrt(b^2 + (XS + (Ct/2) - (Cr/2))^2);

% Normal force on n fins
CnaF = beta*(b/d)^2 /(1 + sqrt( 1 + (2*LF/(Cr + Ct))^2 ) );

% Normal force on body
CnaFB = Kfb * CnaF;

% Center of Press. of fins
xF = XF + (XS/3)*(Cr + 2*Ct)/(Cr + Ct) +...
    (1/6)*(Cr + Ct - (Cr*Ct)/(Cr + Ct));


%% Lifting Body

% Body length
Lb = (XF - Ln) + Cr;       
%Lb = rocket.geo.body.L/12; % ft
% Normal Coeff. of Body
CnaB = (4/pi) .* (Lb/d).* AoA * pi/180;

% Location of Normal Force of Body
xB = Ln + (Lb/2); % ft

%% Total Cp

% Total normal force on rocket
CnaT = CnaN + CnaFB + CnaB;

% Center of Press.
xCP = (CnaN*xN + CnaFB*xF + CnaB*xB)./CnaT;

rocket.data.aero.cp = xCP*12; % in
%rocket.aero.nc.SA = SA_nose;
rocket.data.aero.cp_aoa = AoA;

end

