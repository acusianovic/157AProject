tic
%clc; clear variables; close all;
% 157A Project
fprintf('Optimization Started \n')
stable = 0;
g = 0; % good rockets
b = 0; % bad rockets
n = 0;
vg = 0;
numGoodRockets = 100;
resultRockets = struct(['Good','Bad'],{});
% while we have less than (n) good rockets:
fprintf('Finding good rockets... \n')

boolArray = zeros(1,3);
while  g < numGoodRockets
    fprintf('.')
    if mod(n,100) == 0
        fprintf('\n')
    end
    
    newRocket = rocket();
    newRocket = getRandomRocket(newRocket); % initialize random rocket
    newRocket = getPropulsionDetails(newRocket); % initialize rocket propulsion
    newRocket = getWeightLength(newRocket); % estimate rocket weight and length
    %% TODO: add aerodynamics, rn just assumes Cd = 0.7
    %newRocket = aerodynamics(newRocket); % get aero coefficients vs mach number
    newRocket.aero.Cd = 0.7;
    %% TODO: get center of pressure from aero
    %newRocket = getCP(newRocket); % get center of pressure versus mach number
    %% TODO: get CGs and inertias to use for stability
    %newRocket = getCG(newRocket); % get CG versus propellant weight
    %newRocket = getInertias(newRocket); % get center of pressure versus mach number and propellant weight

    newRocket = oneDOFflightTrajectory(newRocket); % get trajectory, apogee, and OTRS
    
    % calculate rocket stability parameters (static margin)
    %TODO: newRocket = stability(newRocket);
    
    % check rocket against requirements OTRS, apogee, total impulse
    [goodBool, b_arr] = isGood(newRocket);
    if goodBool   
       resultRockets(g+1).Good = newRocket; %   store plane if above is good
       g = g+1;
       fprintf('GOOD')
    else
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
boolArray = n - boolArray;
figure
bar(boolArray)
set(gca,'xticklabel',{'apogeeBad', 'otrsBad', 'arBad'})

%% initialize data variables
W = zeros(g,1);
m_p = zeros(g,1);
m_d = zeros(g,1);
m_w = zeros(g,1);
m_propulsion = zeros(g,1);

apogee = zeros(g,1);
OTRS = zeros(g,1);
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
   L(n) =  resultRockets(n).Good.data.length.L;
   D(n) = resultRockets(n).Good.geo.body.D;
   LD(n) = resultRockets(n).Good.geo.LD;
   
   Thrust(n) = resultRockets(n).Good.prop.F;
   OF(n) = resultRockets(n).Good.prop.OF;
   PC(n) = resultRockets(n).Good.prop.PC;
    
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
L = L(wI);
D = D(wI);

fin_S = fin_S(wI);
fin_AR = fin_AR(wI);
fin_b = fin_b(wI);
fin_c = fin_c(wI);
fin_LE = fin_LE(wI);
fin_sweep = fin_sweep(wI);
fin_TR = fin_TR(wI);


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
% figure
% x = b;
% y = L;
% z = v_cruise;
% qx = linspace(min(x(:,1)),max(y),50);
% qy = linspace(min(x(:,1)),max(y),50);
% F = scatteredInterpolant(x(:,1),y,z(:,1));
% [Xq,Yq] = meshgrid(qx, qy);
% F.Method = 'natural';
% Z = F(Xq,Yq);
% meshc(Xq,Yq,Z)
% zlabel('Cruise Speed ft/s')
% xlabel('Span, ft')
% ylabel('Length, ft')
% shading interp
% set(gca, 'FontSize', 17, 'FontWeight', 'bold')


%%
figure
bar(1:g,OTRS)
ylabel('Off the rail speed, ft/s')

%%
figure
bar(1:g,m_p)
ylabel('Propellant Mass, lb')

%%
figure
bar(1:g,m_w)
ylabel('Propellant Mass, lb')

%%
figure
bar(1:g,L./12)
ylabel('Rocket Length, ft')

%%
figure
bar(1:g,D)
ylabel('Rocket Diameter, in')

%%
figure
bar(1:g,L)
ylabel('Length, ft')



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
valf = (R./min(R)).*(ROC./min(ROC)).*(max(W)./W);
figure
bar(1:g,valf)

%%
figure
hold on
scatter(LD,apogee./5280)
yline(62,'LineWidth',3)

%%
figure
hold on
stem3(LD,m_w,apogee./5280);
xlabel('Aspect Ratio')
ylabel('Wet Mass lbm')
zlabel('Apogee miles')
xlim([15 25])
