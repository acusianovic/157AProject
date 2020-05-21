P = 100:25:600;
h = 0:10000:70000;
apogee = zeros(1,length(h));
OTRS = zeros(1,length(h));
SM_dry = zeros(1,length(d));
SM_wet = zeros(1,length(d));
for j = 1:length(P)
    rocket = betsyMK4;
    rocket.prop.PC = P(j);
    rocket = getPropulsionDetails(rocket,atmo_dat);
    rocket = getWeightLength(rocket);
    rocket = getCG(rocket);
    rocket = getCP(rocket);
    rocket = oneDOFflightTrajectory(rocket,atmo_dat);
    rocket = stability(rocket);
    apogee(j) = rocket.data.performance.apogee;
    OTRS(j) = rocket.data.performance.OTRS;
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
plot(P,apogee/5280,'LineWidth',2)
grid on
yline(62,'b--','LineWidth',2);
ylabel('Apogee, miles')
yyaxis right
plot(P,OTRS,'LineWidth',2)
ylabel('Off-the-rail Speed, ft/s')
yline(100,'r--','LineWidth',2);
xlabel('Chamber Pressure psi')
%%
figure
plot(P,SM_dry,P,SM_wet)
legend('dry','wet')


