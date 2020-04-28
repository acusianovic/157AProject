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
% stabilityGood = false;
% %rocket.data.stability.yaw_is_stable = 1;
% if rocket.data.stability.is_stable && rocket.data.stability.yaw_is_stable 
%     stabilityGood = true;
%     %rocket.data.stability.stall = 0;
%     if rocket.data.stability.stall
%         stabilityGood = false;
%     end
% end



%% geometry good
if rocket.geo.LD <= 25
    arGood = true;
else
    arGood = false;
end
%arGood = true;
    

%% Overall Good
rocketGood = false;

if apogeeGood && otrsGood && arGood
    rocketGood = true;
end

goodArray = [apogeeGood, otrsGood, arGood];

end


