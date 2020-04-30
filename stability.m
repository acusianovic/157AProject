function rocket = stability(rocket)

    % static stability with wet mass (Cp-Cg)/D
    % static stability with dry mass (Cp-Cg)/D
    aoa = rocket.aero.cp_aoa;
    SM_dry = (rocket.aero.cp - rocket.data.CG.dry)./rocket.geo.body.D;
    SM_wet = (rocket.aero.cp - rocket.data.CG.wet)./rocket.geo.body.D;
    
    plt = 0;
    if plt
       figure 
       plot(aoa,SM_dry,aoa,SM_wet,'LineWidth',2);
       xlabel('Angle of Attack, degrees')
       ylabel('Stability Margin')
       yline(1,'--','LineWidth',2);
       legend('Dry','Wet')
       grid on
        
    end
    rocket.aero.SM_dry = SM_dry(1);
    rocket.aero.SM_wet = SM_wet(1);
    
    
end

