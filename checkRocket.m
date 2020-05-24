rocket = betsyMK5;
%rocket.prop.PC = P(j);
rocket = getPropulsionDetails(rocket,atmo_dat);
rocket = getWeightLength(rocket);
rocket = getCG(rocket);
rocket = getCP(rocket);
rocket = checkFlutter(rocket);
rocket = oneDOFflightTrajectory(rocket,atmo_dat);
rocket = stability(rocket);
[goodBool, b_arr] = isGood(rocket);

goodBool