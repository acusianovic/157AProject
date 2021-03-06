function rocket = oneDOFflightTrajectory(rocket,atmo_dat)
% Simulate rocket in 1D to get apogee, OTRS

% simulation parameters
simTime = 300;
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

% Cd = rocket.aero.Cd;
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
a_arr = zeros(N,1);
q_arr = zeros(N,1);

% Initial conditions
v=0;y=0;x=0;dv=0;T=0;D=0;M=0;a=0;q=0;

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
    a_arr(i)=a;
    q_arr(i)=q;
    nu = lininterp1(atmo_dat.Z_L,atmo_dat.nu,y); % kinematic viscosity, ft2/s
    sos = lininterp1(atmo_dat.Z_L,atmo_dat.c,y); % speed of sound, ft/s
    P_a = lininterp1(atmo_dat.Z,atmo_dat.P,y); % psi
    rho = lininterp1(atmo_dat.Z,atmo_dat.rho,y); % slug/ft3
    
    M = abs(v/sos);
    Cd(i) = getDrag2(rocket,v,sos,nu);
    
%     if M < 7
%         %Cd = lininterp1(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),M);
%     else
%         %Cd = min(Aerobee150ADragData(2,:));
%     end
    % Physics
    q = 0.5*rho*v^2; % psf
    D = q*S*Cd(i); % drag, lbf
    if v >= 0 && t(i) <= t_b
    %% Thrust period
        dm = mdot*dt;
        T = mdot*cstar*getThrustCoefficient(rocket,P_a)/g;
        a = (T-D-(m/g)*g_gr)/(m/g);
        dv = a*dt;
        % track off the rail speed
        if y <= launch_tower_height
            OTRS = v;
        end
        apind = i;
        
        
    elseif v >= 0
    %% Coast period
        T = 0; dm = 0;
        a = (T-D-(m/g)*g_gr)/(m/g);
        dv = a*dt;

    elseif v < 0
    %% Descent period
        %D_p = 0.5*rho*v^2*S_p*C_dp; % parachute drag
        % no parachute for now
        a = (D-(m/g)*g_gr)/(m/g);
        dv = a*dt;
        break;
        
    end
    % Update mass, velocity, and position
    m = m - dm;
    v = v + dv;
    y = y + v*cosd(0)*dt;
    x = x + v*sind(0)*dt;

    t_land = t(i);

    
    % If rocket hits the ground, stop the simulation
    if y < -0.1
        break;
    end
end
if 0
    figure
    plot(t,y_arr./5280,'LineWidth',2)
    ylabel('Height, miles')
    xlabel('Time, s')
    grid on
    xlim([0 260.5])
    set(gca, 'FontSize', 11, 'FontWeight', 'bold')

    %%
    figure
    plot(t,T_arr,t,D_arr,'LineWidth',2)
    legend('Thrust','Drag')
    xlim([0 134.5])
    grid on
    %%
    figure
    plot(t,v_arr)
    %%
    figure
    plot(t,M_arr)
    
    %%
    figure
    plot(t,a_arr/g,'LineWidth',2)
    grid on
    xlabel('Time(s)');ylabel("Acceleration, g's")
    
    %%
    figure
    plot(t,q_arr/144,'LineWidth',2)
    grid on
end
%% Performance
[apogee, ind] = max(y_arr);

rocket.prop.F_opt = lininterp1(y_arr,T_arr,rocket.prop.expansion_h);

rocket.prop.F_mean = mean(T_arr(1:apind));
rocket.data.performance.OTRS = OTRS;
rocket.data.performance.apogee = apogee;
rocket.data.performance.Mmax = max(M_arr); % mach number
rocket.data.performance.Vmax = max(v_arr); % ft/s
rocket.data.performance.maxg = max(a_arr)/g;
rocket.data.performance.maxQ = max(q_arr)/144;
% rocket.data.performance.height = y_arr;
% rocket.data.aero.Cd = Cd;
% rocket.data.aero.Mach = Ma;

end