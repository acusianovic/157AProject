tic
%clc; clear variables; close all;
% 157A Project
fprintf('Optimization Started \n')
stable = 0;
g = 0; % good rockets
b = 0; % bad rockets
n = 0;
vg = 0;
numGoodRockets = 25;
resultRockets = struct(['Good','Bad'],{});
% while we have less than (n) good rockets:
fprintf('Finding good rockets... \n')

boolArray = zeros(1,4);
while  g < numGoodRockets
    fprintf('.')
    if mod(n,100) == 0
        fprintf('\n')
    end
    
    newRocket = rocket();
    newRocket = getRandomRocket(newRocket); % initialize random rocket
    newRocket = getPropulsionDetails(newRocket); % initialize rocket propulsion
    newRocket = getWeightLength(newRocket); % estimate rocket weight and length
    newRocket = getCP(newRocket); % get center of pressure versus angle of attack
    newRocket = getCG(newRocket); % get CG for dry mass and wet mass
    %newRocket = getInertias(newRocket); % get center of pressure versus mach number and propellant weight

    newRocket = oneDOFflightTrajectory(newRocket); % get trajectory, apogee, and OTRS
    newRocket = stability(newRocket); % static stability
   
    % calculate rocket stability parameters (static margin)
    %TODO: newRocket = stability(newRocket);
    
    % check rocket against requirements OTRS, apogee, total impulse
    [goodBool, b_arr] = isGood(newRocket);
    if goodBool   
       resultRockets(g+1).Good = newRocket; %   store plane if above is good
       g = g+1;
       fprintf('GOOD')
    else
        %resultRockets(b+1).Bad = newRocket;
        b = b+1;
    end
    n = n + 1;
    boolArray = boolArray + b_arr;
end
%%
beep
fprintf('\n\n%d good rockets found \n',g)
fprintf('%d bad rockets discarded \n',b)
%%
badfails = n - boolArray;
figure
bar(badfails)
set(gca,'xticklabel',{'apogeeBad', 'otrsBad', 'arBad','stabilityBad'})

%% initialize data variables
W = zeros(g,1);
m_p = zeros(g,1);
m_d = zeros(g,1);
m_w = zeros(g,1);
m_propulsion = zeros(g,1);

apogee = zeros(g,1);
OTRS = zeros(g,1);
M = zeros(g,1);
L = zeros(g,1);
D = zeros(g,1);
fin_S = zeros(g,1);
fin_AR = zeros(g,1);
fin_b = zeros(g,1);
fin_c = zeros(g,1);
fin_sweep = zeros(g,1);
fin_LE = zeros(g,1);
fin_TR = zeros(g,1);

OF = zeros(g,1);
PC = zeros(g,1);
Thrust = zeros(g,1);
t_b = zeros(g,1);
Isp = zeros(g,1);
Itot = zeros(g,1);

CL = zeros(100,g);
CD = zeros(100,g);
D1 = zeros(100,g);
LD = zeros(g,1);

% extract data for n planes
for n = 1:g
   m_p(n) = resultRockets(n).Good.prop.m_p;
   m_d(n) = resultRockets(n).Good.data.weight.dry;
   m_w(n) = resultRockets(n).Good.data.weight.wet;
   m_propulsion(n) = resultRockets(n).Good.data.weight.propulsion;
   
   apogee(n) =  resultRockets(n).Good.data.performance.apogee; % ft
   OTRS(n) = resultRockets(n).Good.data.performance.OTRS;
   M(n) = resultRockets(n).Good.data.performance.Mmax;
   L(n) =  resultRockets(n).Good.data.length.L;
   D(n) = resultRockets(n).Good.geo.body.D;
   LD(n) = resultRockets(n).Good.geo.LD;
   
   Thrust(n) = resultRockets(n).Good.prop.F;
   t_b(n) = resultRockets(n).Good.prop.t_b;
   OF(n) = resultRockets(n).Good.prop.OF;
   PC(n) = resultRockets(n).Good.prop.PC;
   Isp(n) = resultRockets(n).Good.prop.Isp;
   Itot(n) = resultRockets(n).Good.prop.Itot;
%    fin_S(n) =  resultRockets(n).Good.geo.fin.S;
%    fin_AR(n) = resultRockets(n).Good.geo.fin.AR;
%    fin_b(n) =  resultRockets(n).Good.geo.fin.b;
%    fin_c(n) =  resultRockets(n).Good.geo.fin.c;
%    fin_LE(n) =  resultRockets(n).Good.geo.fin.LE;
%    fin_sweep(n) =  resultRockets(n).Good.geo.fin.sweep;
%    fin_TR(n) =  resultRockets(n).Good.geo.fin.TR;
   
  
end

[m_w, wI] = sort(m_w);
dummy = resultRockets;
for n = 1:g
    resultRockets(n).Good = dummy(wI(n)).Good;
end

apogee = apogee(wI);
OTRS = OTRS(wI);
M = M(wI);
L = L(wI);
D = D(wI);
m_p = m_p(wI);
m_w = m_w(wI);
m_propulsion = m_propulsion(wI);
m_d = m_d(wI);
Thrust = Thrust(wI);
t_b = t_b(wI);
Isp = Isp(wI);
OF = OF(wI);
PC = PC(wI);
    

fin_S = fin_S(wI);
fin_AR = fin_AR(wI);
fin_b = fin_b(wI);
fin_c = fin_c(wI);
fin_LE = fin_LE(wI);
fin_sweep = fin_sweep(wI);
fin_TR = fin_TR(wI);

%% Visualize spread and find the best rocket
if 0 % set to 1 to plot after running
%%
figure
bar(1:g,m_w)
ylabel('Wet Mass, lb')
%%
figure
bar(1:g,m_p)
ylabel('Propellant Mass, lb')

%%
figure
bar(1:g,m_p./m_d)
ylabel('Mass Fraction')

%%
figure
bar(1:g,Thrust)
ylabel('Thrust')

%%
figure
bar(1:g,M)
ylabel('Max Mach Number')

%%
figure
bar(1:g,OTRS)
ylabel('Off the rail speed, ft/s')

%%
figure
bar(1:g,apogee./5280)
ylabel('Apogee, miles')

%%
figure
bar(1:g,PC)
ylabel('Chamber Pressure, psi')

%%
figure
bar(1:g,Itot)
ylabel('Total Impulse, lbf-s')
yline(200000,'--','LineWidth',2);

%%
figure
bar(1:g,L./12)
ylabel('Rocket Length, ft')

%%
figure
bar(1:g,LD)
ylabel('Aspect Ratio')
%%
figure
bar(1:g,D)
ylabel('Rocket Diameter, in')

%%
figure
plot(D, apogee./5280,'o','LineWidth',2)
xlabel('Diameter, in')
ylabel('Apogee, miles')

%%
% Higher PC -> higher tank and engine weight
figure
plot(PC, m_propulsion,'o','LineWidth',2)
xlabel('Chamber Pressure, psi'); ylabel('Prop System Mass, lbm')

%% Higher PC -> greater Isp
figure
plot(PC, Isp,'o','LineWidth',2)
xlabel('Chamber Pressure, psi'); ylabel('Specific Impulse, s')
grid on

%% Higher PC -> less propellant needed
figure
plot(PC, m_p./Itot,'o','LineWidth',2)
xlabel('Chamber Pressure, psi'); ylabel('Propellant Mass/Total Impulse, 1/s')
grid on

%%
figure
plot(PC, m_w,'o','LineWidth',2)
xlabel('Chamber Pressure, psi'); ylabel('Wet Mass, lbm')

%%
figure
plot(Thrust./m_w, apogee,'o','LineWidth',2)
xlabel('Chamber Pressure, psi'); ylabel('Mass Fraction, dim')

%%
figure
plot(m_propulsion, apogee./5280,'o','LineWidth',2)
xlabel('Chamber Pressure, psi'); ylabel('Apogee, ')

%%
figure
plot(LD, apogee./5280,'o','LineWidth',2)
xlabel('LD'); ylabel('Apogee, miles')

%%
figure
hold on
stem3(LD,m_w,apogee./5280);
xlabel('Aspect Ratio')
ylabel('Wet Mass lbm')
zlabel('Apogee miles')
%xlim([19 25])

%%
figure
hold on
stem3(LD,PC,m_w);
xlabel('Length, ft')
ylabel('Diameter, in')
zlabel('Wet Mass, lbm')

%%
% figure
% x = v_cruise;
% y = R./5280;
% z = ROC.*60;
% qx = linspace(min(x(:,1)),max(x(:,1)),100); %picked the first column but this should be fixed more properly
% qy = linspace(min(y),max(y),100);
% 
% F = scatteredInterpolant(x(:,1),y,z);
% [Xq,Yq] = meshgrid(qx, qy);
% F.Method = 'natural';
% Z = F(Xq,Yq);
% meshc(Xq,Yq,Z)
% xlabel('Cruise Speed ft/s')
% zlabel('Rate of Climb, fpm')
% ylabel('Range, miles')
% shading interp
% set(gca, 'FontSize', 17, 'FontWeight', 'bold')
%%
figure
x = D;
y = PC;
z = m_w;
qx = linspace(min(x),max(x),100);
qy = linspace(min(y),max(y),100);
F = scatteredInterpolant(x,y,z);
[Xq,Yq] = meshgrid(qx, qy);
F.Method = 'natural';
Z = F(Xq,Yq);
meshc(Xq,Yq,Z)
xlabel('Diameter, in')
ylabel('Chamber Pressure, psi')
zlabel('Wet Mass, lbm')
zlim([500 800])
shading interp
set(gca, 'FontSize', 17, 'FontWeight', 'bold')

%%
figure
bar(1:g,Thrust)


%%
% figure
% hold on
% for n = 1:g
%     plot(v_ref,D1(:,n))
% end
% eta = resultPlanes(1).Good.prop.eta_p; % eta and hp same for all planes so I just used the first one
% hp = resultPlanes(1).Good.prop.hp;
% thrust = (hp * 550 * eta) ./ v_ref;
% plot(v_ref,thrust);
% hold off
toc
%%
%% max value function
valf = (apogee./min(apogee)).*(OTRS./min(OTRS)).*(max(m_w)./m_w);
figure
bar(1:g,valf)

%%
figure
hold on
scatter(LD,apogee./5280)
yline(62,'LineWidth',3)


%% To plot the rockets on a 3dplot
% N=vg;%vg;
% hf=figure('units','normalized','outerposition',[0 0 1 1]);
% hf.ToolBar='none';
% nS   = sqrt(N);
% nCol = ceil(nS);
% nRow = nCol - (nCol * nCol - N > nCol - 1);
% for k = 1:N
%   subplot(nRow, nCol, k);
%   plotPlaneGeo(resultRockets(k).Good);
%   set(gca,'XTick',[], 'YTick', [], 'ZTick', [])
%   title(k)
% end
end

