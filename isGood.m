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
if rocket.data.aero.SM_dry >= 1.25 && rocket.data.aero.SM_wet >= 1.25
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

%% Flutter or not
if rocket.data.aero.flutter > 1.5*rocket.data.performance.Vmax
   finGood = true;
else
    finGood = false;
end


%% Overall Good
rocketGood = false;

if apogeeGood && otrsGood && arGood && stabilityGood && finGood
    rocketGood = true;
end

goodArray = [apogeeGood, otrsGood, arGood, stabilityGood, finGood];

end


