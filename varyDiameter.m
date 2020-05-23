P = 100:25:400;
h = 0:10000:70000;
d = 12:1:16;
apogee = zeros(1,length(d));
OTRS = zeros(1,length(d));
Isp = zeros(1,length(d));
eps = zeros(1,length(d));
F_mean = zeros(1,length(d));
F_opt = zeros(1,length(d));
mdot = zeros(1,length(d));
LD = zeros(1,length(d));
SM_dry = zeros(1,length(d));
SM_wet = zeros(1,length(d));
load('atmo_dat.mat','atmo_dat');
for j = 1:length(d)
    rocket = betsyMK4;
    
    %rocket.prop.PC = P(j);
    %rocket.prop.expansion_h = h(j);
    rocket.geo.body.D = d(j);
    rocket = getPropulsionDetails(rocket,atmo_dat);
    rocket = getWeightLength(rocket);
    rocket = getCG(rocket);
    rocket = getCP(rocket);
    rocket = oneDOFflightTrajectory(rocket,atmo_dat);
    rocket = stability(rocket);
    apogee(j) = rocket.data.performance.apogee;
    OTRS(j) = rocket.data.performance.OTRS;
    Isp(j) = rocket.prop.Isp;
    eps(j) = rocket.prop.eps;
    F_mean(j) = rocket.prop.F_mean;
    mdot(j) = rocket.prop.mdot;
    F_opt(j) = rocket.prop.F_opt;
    LD(j) = rocket.geo.LD;
    SM_dry(j) = rocket.data.aero.SM_dry;
    SM_wet(j) = rocket.data.aero.SM_wet;

%     fprintf('\nApogee: %g miles\n',rocket.data.performance.apogee/5280)
%     fprintf('OTRS: %g ft/s\n',rocket.data.performance.OTRS)
%     fprintf('Static Stability Margin (wet): %g\n',rocket.aero.SM_wet)
%     fprintf('Static Stability Margin (dry): %g\n',rocket.aero.SM_dry)
%     fprintf('Wet Weight: %g lbm\n',rocket.data.weight.wet)
%     fprintf('Dry Weight: %g lbm\n',rocket.data.weight.dry)
end
%%
figure
hold on
plot(d,apogee/5280,'LineWidth',2)
grid on
yline(62,'b--','LineWidth',2);
ylabel('Apogee, miles')
yyaxis right
plot(d,OTRS,'LineWidth',2)
ylabel('Off-the-rail Speed, ft/s')
yline(100,'r--','LineWidth',2);
xlabel('Diameter')
%%
figure
plot(d,LD)
xlabel('diameter');ylabel('LD')
yline(18);


%%
figure
plot(d,Isp,'LineWidth',2)
ylabel('Isp')
yyaxis right
plot(d,eps,'LineWidth',2)
ylabel('eps')
%%
figure
plot(d,mdot,'LineWidth',2)
ylabel('mdot')
yyaxis right
plot(d,F_mean,'LineWidth',2)
ylabel('F')
%%
figure
plot(d,F_opt)
%%
figure
plot(LD,SM_dry,LD,SM_wet)
legend('dry','wet')
