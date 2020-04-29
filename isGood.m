function [rocketGood, goodArray] = isGood(rocket)
%% Apogee Requirement
if rocket.data.performance.apogee >= 333000
    apogeeGood = true;
else
    apogeeGood = false;
end


%% OTRS Requirement (Off-the-rail speed)
if rocket.data.performance.OTRS >= 100
    otrsGood = true;
else
    otrsGood = false;
end

%% isStable
if rocket.aero.SM_dry >= 1.25 && rocket.aero.SM_wet >= 1.25
    stabilityGood = true;
else
    stabilityGood = false;
end
    

%% geometry good
if rocket.geo.LD <= 20
    arGood = true;
else
    arGood = false;
end
%arGood = true;
    

%% Overall Good
rocketGood = false;

if apogeeGood && otrsGood && arGood && stabilityGood
    rocketGood = true;
end

goodArray = [apogeeGood, otrsGood, arGood, stabilityGood];

end


