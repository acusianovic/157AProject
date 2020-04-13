function [planeGood, goodArray] = isGood(plane)
%% identifying good RoC
if plane.data.performance.ROC >= 33
    rocGood = true;
else
    rocGood = false;
end


%% V_cruise good
if plane.data.aero.v_cruise(1) >= 220 && plane.data.aero.v_cruise(2) >= 220
    VcGood = true;
else
    VcGood = false;
end

%% Range
if plane.data.performance.R >= 500*5280%% && plane.data.performance.R <= 600*5280
    rangeGood = true;
else
    rangeGood = false;
end

%% Need to be able to fly at at least stall speed ADD ANGLE OF ATTACKS
if plane.prop.thrust(1) - plane.data.aero.D(1,1) >= 0
    minSpeedGood = true;
else
    minSpeedGood = false;
end

%% isStable
stabilityGood = false;
%plane.data.stability.yaw_is_stable = 1;
if plane.data.stability.is_stable && plane.data.stability.yaw_is_stable 
    stabilityGood = true;
    %plane.data.stability.stall = 0;
    if plane.data.stability.stall
        stabilityGood = false;
    end
end



%% geometry good
geo = plane.geo;
GeoIsGood = false;
    GeoWingTailLocationGood = false;    
    GeoAreaRatioGood = false;
    GeoFitsRetardent = false;    
    GeoFitsFuel = false;
    % location of wing, tails are ok
    if all([geo.wing.LE, geo.h_tail.LE, geo.v_tail.LE] > 0)...
    && all([geo.wing.LE+geo.wing.c, geo.h_tail.LE+geo.h_tail.c, geo.v_tail.LE+geo.v_tail.c] < geo.body.L)...
    && all([geo.h_tail.LE, geo.v_tail.LE] > geo.wing.LE+geo.wing.c)
        GeoWingTailLocationGood = true;
    end
    % check wing area ratios
    if geo.wing.S > geo.h_tail.S && geo.wing.S > geo.v_tail.S
        GeoAreaRatioGood = true;
    end
    % fits retardent
    if (geo.body.L * 0.25*pi*geo.body.D^2 > 256 * 2) %fuselage volume at least 2x water volume
        GeoFitsRetardent = true;
    end
    % fits fuel in wing
    wingVolume = 0.5*geo.wing.c*geo.wing.b*(geo.wing.TR+1) * 0.0815662901; % number from area of airfoil per chord length
    if plane.prop.fuel_volume < wingVolume % Allow half volume margin for wing "fuel box"
        GeoFitsFuel = true;
    end
% Overall geo check
if all([GeoWingTailLocationGood, GeoAreaRatioGood, GeoFitsRetardent, GeoFitsFuel])
    GeoIsGood = true;
end
    
%% Real
real = false;
if isreal(plane.data.aero.CD) && isreal(plane.data.aero.CL)
    real = true;
end

%% Overall Good
planeGood = false;

%stabilityGood = 1;
if GeoIsGood && rocGood && VcGood && rangeGood && stabilityGood && real && minSpeedGood
    planeGood = true;
end

goodArray = [GeoIsGood, rocGood, VcGood, rangeGood, stabilityGood, real, minSpeedGood];

end


