function [rocket] = oneDOFflightTrajectory(rocket)
% Simulate rocket in 1D to get apogee, OTRS

% simulation parameters
simTime = 500;
dt = 0.01;
N = simTime/dt;
t = 0:dt:simTime-dt;
load('Aerobee150ADragData.mat','Aerobee150ADragData');

launch_tower_height = 160;
m = rocket.data.weight.wet;  % total weight
mdot = rocket.prop.mdot; % mass flow rate, lbm/s
t_b = rocket.prop.t_b;  % burn time, s
cstar = rocket.prop.cstar*3.28; % characteristic velocity, ft/s
S = pi/4*rocket.geo.body.D^2/144; % reference area, ft2

Cd = rocket.aero.Cd;
g = 32.174;
R_e = 3.67E6*3.28; % earth radius, ft

% Intialize data arrays
y_arr = zeros(N,1);
x_arr = zeros(N,1);
v_arr = zeros(N,1);
T_arr = zeros(N,1);
D_arr = zeros(N,1);
M_arr = zeros(N,1);
rho_arr = zeros(N,1);

% Initial conditions
v=0;y=0;x=0;dv=0;T=0;D=0;M=0;

% Begin simulation
for i = 1:N
    % gravity
    g_gr = g*(R_e/(R_e+y))^2;
    
    % store variables
    y_arr(i)=y;
    x_arr(i)=x;
    v_arr(i)=v;
    T_arr(i)=T;
    D_arr(i)=D;
    M_arr(i)=M;
    
    %[rho,P_a,~] = getAtm(y,0); % slug/ft3, psi
    [~, a, P_a, rho] = atmos(y/3.28);
    rho = rho/515.379; % slug/ft3
    P_a = P_a/101325*14.7; % psi
    M = abs(v/(a*3.28));
    if M < 7
        Cd = lininterp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),M);
    else
        Cd = min(Aerobee150ADragData(2,:));
    end
    % Physics
    D = 0.5*rho*v^2*S*Cd; % drag, lbf
    if v >= 0 && t(i) <= t_b
    %% Thrust period
        dm = mdot*dt;
        T = mdot*cstar*getThrustCoefficient(rocket,P_a)/g;
        dv = (T-D-(m/g)*g_gr)/(m/g)*dt;
        % track off the rail speed
        if y <= launch_tower_height
            OTRS = v;
        end
        
        
    elseif v >= 0
    %% Coast period
        T = 0; dm = 0;
        dv = (T-D-(m/g)*g_gr)/(m/g)*dt;

    elseif v < 0
    %% Descent period
        %D_p = 0.5*rho*v^2*S_p*C_dp; % parachute drag
        % no parachute for now
        dv = (D-(m/g)*g_gr)/(m/g)*dt;
        break;
        
    end
    % Update mass, velocity, and position
    m = m - dm;
    v = v + dv;
    y = y + v*cosd(0)*dt;
    x = x + v*sind(0)*dt;

    t_land = t;
    % If rocket hits the ground, stop the simulation
    if y < -0.1
        break;
    end
end
if 0
    figure
    plot(t,y_arr./5280)
    %%
    figure
    plot(t,T_arr,t,D_arr)
    legend('Thrust','Drag')
    %%
    figure
    plot(t,v_arr)
    %%
    figure
    plot(t,M_arr)
end
%% Performance
[apogee, ind] = max(y_arr);

rocket.data.performance.OTRS = OTRS;
rocket.data.performance.apogee = apogee;


end