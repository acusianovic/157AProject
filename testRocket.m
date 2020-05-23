P = 100:25:400;
h = 0:10000:70000;
d = 10:0.5:14;
apogee = zeros(1,length(h));
OTRS = zeros(1,length(h));
Isp = zeros(1,length(h));
eps = zeros(1,length(h));
F_mean = zeros(1,length(h));
F_opt = zeros(1,length(h));
mdot = zeros(1,length(h));
load('atmo_dat.mat','atmo_dat');
for j = 1:length(h)
    rocket = betsyMK4;
    
    %rocket.prop.PC = P(j);
    rocket.prop.expansion_h = h(j);
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
plot(h,apogee/5280,'LineWidth',2)
grid on
yline(62,'b--','LineWidth',2);
ylabel('Apogee, miles')
yyaxis right
plot(h,OTRS,'LineWidth',2)
ylabel('Off-the-rail Speed, ft/s')
yline(100,'r--','LineWidth',2);
xlabel('Expansion Altitude, ft')
xlim([0 40000])
%%
figure
plot(h,Isp,'LineWidth',2)
ylabel('Isp')
yyaxis right
plot(h,eps,'LineWidth',2)
ylabel('eps')
%%
figure
plot(h,mdot,'LineWidth',2)
ylabel('mdot')
yyaxis right
plot(h,F_mean,'LineWidth',2)
ylabel('F')
%%
figure
plot(h,F_opt)
