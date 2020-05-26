function [x,D,Lc,Lf,Ln] = getEngineGeometry(Vc,Ac,At,Ae,theta,dx)
    Rc = sqrt(Ac/pi); % chamber radius, m
    Rt = sqrt(At/pi); % throat radius, m
    Re = sqrt(Ae/pi); % exit radius, m
    
    rc1 = 0.025; % frustum entry radius of curvature, m
    rc2 = 1.5*Rt; % throat converging radius of curvature, m
    rc3 = 0.382*Rt; % throat diverging radius of curvature, m
    
    
    global Xe Ye alpha_n alpha_e
    
    alpha = 15; % expansion angle, deg
    alpha_n = 22.3; %Degrees
    alpha_e = 13.5; %Degrees
    
    Lnc = (Rt*(sqrt(Ae/At)-1)+Rt/2*(secd(alpha)-1))/tand(alpha); % conical nozzle length, m
    Ln = 0.8*Lnc; % bell nozzle length, m
    Ye = Re-Rt-0.382*Rt*(1-cosd(alpha_n)); % Y-Coordinate at Pt.E, m
    Xe = Ln-0.382*Rt*sind(alpha_n); % X-Coordinate at Pt.E, m
    
    fun = @RaoFx;
    X0 = [0.984,-0.324,-0.373,0.105]; % initial guess
    X = fsolve(fun,X0);
    P = X(1); Q = X(2); S = X(3); T = X(4); % bell nozzle parameters solution
    
    Lf = (Rc-Rt)*tand(theta); % frustum length (first order estimate), m
    Lc = (Vc-1/3*Lf*(Ac^2+At^2+sqrt(Ac*At)))/Ac; % chamber length, m

    Lc = round(Lc,3); % rounded chamber length, m
    x0 = Lc;
    x1 = round(x0+rc1*sind(theta),3); y1 = Rc-rc1*(1-cosd(theta));
    y2 = Rt+rc2*(1-cosd(theta)); x2 = round(x1+(y1-y2)/tand(theta),3);
    xt = round(x2+rc2*sind(theta),3);
    x3 = round(xt+rc3*sind(alpha_n),3); y3 = Rt+rc3*(1-cosd(alpha_n));
    ye = Re; xe = round(x3+Xe,3);
    Lf = xt-x0; % frustum length, m
    Ln = xe-xt; % nozzle length, m

    x = 0:dx:xe; % downstream location array, m
    xarray1 = 0:dx:x0;
    xarray2 = x0:dx:x1;
    xarray3 = x1:dx:x2;
    xarray4 = x2:dx:xt;
    xarray5 = xt:dx:x3;
    xarray6 = x3:dx:xe;
    yarray1 = Rc*ones(1,length(xarray1));
    yarray2 = Rc-rc1+sqrt(rc1^2-(xarray2-x0).^2);
    yarray3 = linspace(y1,y2,length(xarray3));
    yarray4 = Rt+rc2-sqrt(rc2^2-(xarray4-xt).^2);
    yarray5 = Rt+rc3-sqrt(rc3^2-(xarray5-xt).^2);
    yarray6 = y3+P.*(xarray6-x3)+Q+(S.*(xarray6-x3)+T).^(1/2);
    D = 2*[yarray1,yarray2(2:end),yarray3(2:end),yarray4(2:end),yarray5(2:end),yarray6(2:end)]; % engine diameter array, m
end