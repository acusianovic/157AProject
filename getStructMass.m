function rocket = getStructMass(rocket)
%% Air Frame
% Material Properties (Carbon Fiber)
E_frame = 13720e3; %psi
rho_frame = 0.051; %lb/in3
strength_yield_frame = 105e3; %psi
G_frame = 603.4e3; %psi
%strength_shear =  %psi

D_i = rocket.geo.body.D; %in
t = 0.075; %in
D_o = D_i + t; %in
R_i = D_i/2; %in
R_o = D_o/2; %in

L = rocket.geo.body.L; %in
I_frame = (pi/4)*(R_o^4 - R_i^4); %in^4
A_frame = pi*(D_o^2 - D_i^2); %in^2
n = 4; %fixed ends
V_frame = (pi/4)*(D_o^2 - D_i^2)*L; 
m_frame = rho_frame*V_frame;

%% Longerons (Al 2024-t3)
E_longeron = 10600e3; %psi
strength_yield_longeron = 50e3; %psi
N_longerons = 4;
w_longeron = 1; %in
t_longeron = 0.1; %in
A_longerons = N_longerons*w_longeron*t_longeron;
L_longeron = L;
I_longeron = (w_longeron*t_longeron^3)/12;
rho_longeron = 0.1;
V_longeron = w_longeron*L_longeron*t_longeron;
m_longerons = N_longerons*rho_longeron*V_longeron;

%% Overall
A = A_frame + A_longerons;

F_crit_yield = (strength_yield_frame * A_frame/A) + (strength_yield_longeron * A_longerons/A); %psi
F_crit_buckle = ((n*pi^2*E_frame*I_frame/(L)^2)*A_frame/A) + ...
    ((n*pi^2*E_longeron*I_longeron/(L_longeron)^2)*A_longerons/A); %psi

m_struct = m_frame + m_longerons;

rocket.data.weight.body = m_struct;

if F_crit_yield < F_crit_buckle
    rocket.data.requirements.criticalForce = F_crit_yield;
else
    rocket.data.requirements.criticalForce = F_crit_buckle;
end