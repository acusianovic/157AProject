
function [plane] = weight_function(plane)



    Weight_guess = 35000;  %lbs, initial weight guess
    W_total = zeros(1,20);
    W_total(1) = Weight_guess;
    Weight = W_total(1);
    for i = 1:20

    %% wing geometry and weight
    S = plane.geo.wing.S;                           %wing area, ft^2   % 
    AR = plane.geo.wing.AR;                         %aspect ratio

    N = 5;%plane.data.N;                      %Ultimate Load Factor (1.5 times limit load factor)(GIVEN)
    sweep_angle = plane.geo.wing.sweep * (3.14/180);            %Deg %Wing 1/4 chord sweep angle
    taper_ratio = plane.geo.wing.TR;     %Taper Ratio
    thickness_ratio_wing = plane.geo.wing.ThR;                  %Maximum Thickness Ratio (GIVEN)
    v_max = plane.data.requirements.v_max*0.593;        %FIX UNITS         %kts   %Equivalent Vmax at SL


    W_wing = 96.948 * ((Weight * N/10^5)^0.65*(AR/cos(sweep_angle))^0.57*(S/100)^0.61*((1 + taper_ratio)/(2*thickness_ratio_wing))^0.36*(1+v_max/500)^0.5)^0.993;


    %% Tail 

    S_horizontal_tail = plane.geo.h_tail.S;   %horizontal tail area, ft^2
    S_vertical_tail = plane.geo.v_tail.S;                  %vertical tail area, ft^2
    b_horizontal_tail = plane.geo.h_tail.b;  %horizontal tail span, ft
    b_vertical_tail = plane.geo.v_tail.b;  %vertical tail span in, ft


    ac_wing = plane.geo.wing.ac;    %ft, distance from wing LE to AC
    chord_horizontal_tail = plane.geo.h_tail.c;      %chord length, horizontal tail, ft
    chord_vertical_tail = plane.geo.v_tail.c;       %chord length, vertical tail, ft
    ac_htail =  plane.geo.h_tail.ac;    %ft, distance from wing leading edge to horizontal tail AC


    %% Horizontal Tail Weight

    %lh = 35 / 12 + (.5 - h_ac_wing) * chord_wing - (.5 - h_ac_htail) * chord_horizontal_tail; %ft       %Distance from Wing MAC to Tail MAC
    lh = ac_htail - ac_wing;
    thr = chord_horizontal_tail*.12*12; %ft      %horizontal tail max root thickness (chord * thick/chord)

    Weight_horizontal_tail = 127*((Weight * N/10^5)^0.87*(S_horizontal_tail/100)^1.2*(lh/10)^0.483*(b_horizontal_tail/thr)^0.5)^0.458; %horizontal tail weight

    %% Vertical Tail Weight

    tvr = chord_vertical_tail*.12*12; %in    %Vertical Tail Max Root Thickness (chord * thick/chord * in/ft)

    Weight_vertical_tail = (2)*  98.5*((Weight * N/10^5)^0.87*(  (.5)*  S_vertical_tail/100)^1.2*(  (.5)*  b_vertical_tail/tvr)^0.5)^0.458;

    %% Fuselage Weight

    length_fuselage = plane.geo.body.L;     %ft       %Fuselage Length
    width_fuselage = plane.geo.body.W;      %ft        %Fuselage Width
    depth_fuselage = plane.geo.body.D;  %ft          %Fuselage Max Depth

    Weight_fuselage = 200*((Weight*N/10^5)^0.286*(length_fuselage/10)^0.857*((width_fuselage+depth_fuselage)/10)*(v_max/100)^0.338)^1.1;


    %% Landing Gear Weight

    Llg = 18; %in     %Length of Main Landing Gear Strut
    N_land = 2;        %Ultimate Load Factor at Wland
    Weight_landing_gear = 0.054*(Llg)^0.501*(Weight*N_land)^0.684;

    %don't need niccolai if we have specific landing gear 
    %Wlg = 100;      %lbs, weight landing gear

    %% TOTAL STRUCTURAL WEIGHT

    W_struct = W_wing + Weight_fuselage + Weight_horizontal_tail + Weight_vertical_tail + Weight_landing_gear;


    %% Total Propulsion Unit (minus Fuel system) Weight

    W_eng = plane.prop.W; %(lbs)     %Bare Engine Weight
    N_eng = plane.prop.numengines;             %# Engines

    W_prop = (2.575*(W_eng)^0.922)*N_eng;    %this equation likely over-estimates propulsion unit weight for a small UAV


    %% Fuel Weight

    W_fuel = plane.prop.fuel_mass;   %(lbs)  

    %% Fuel System Weight

    %rhof = 6.739; %lb/gal fuel mass density JP-8
    %Fg = Wfu / rhof; %gal               %Total Fuel
    %tankint=1; %percent         %Percent of Fuel Tanks that are integral
    %Nt=2;                         %Number of Separate Fuel Tanks
    %Wfs=2.49*((Fg)^0.6*(1/(1+tankint))^0.3*Nt^0.2*Neng^0.13)^1.21

    % specific fuel system weights (fuel tanks, lines) likely can be found for your ai rcraft, if so, use those actual values instead of the niccolai equations.
    Wfs = 100;  %lbs 

    %% Surface Controls Weight

    Wsc = 1.066*Weight^0.626;  

    %% Avionics Weight - use weights of specific sensors you choose

    W_avionics = 1;      


    %% Payload Weight

    W_payload = 16000;    %lbs, weight retardent

    %% TOTAL WEIGHT


    W_total(i) = W_struct + W_prop + Wfs + Wsc + W_payload + plane.prop.fuel_mass + W_avionics;
    Weight = W_total(i);


    end


Weight_total = Weight;

%W_struct = W_wing + Weight_fuselage + W_horizontal_tail + Weight_vertical_tail + Weight_landing_gear;
%Weight_total = W_struct + W_prop + Wfs + Wsc + W_payload + W_fuel + W_avionics;
empty_weight = W_struct + W_prop + Wfs + Wsc + W_avionics;
Weight = zeros(13,1);
Weight(1,1) = W_wing;
Weight(2,1) = Weight_fuselage;
Weight(3,1) = Weight_horizontal_tail;
Weight(4,1) = Weight_vertical_tail;
Weight(5,1) = Weight_landing_gear;
Weight(6,1) = W_prop;
Weight(7,1) = Wfs;
Weight(8,1) = Wsc;
Weight(9,1) = W_avionics;
Weight(10,1) = W_fuel;
Weight(11,1) = W_payload;
Weight(12,1) = empty_weight;
Weight(13,1) = Weight_total;

plane.data.weight.W = Weight;
plane.data.weight.dry = Weight_total - W_payload;
plane.data.weight.wet = Weight_total;
plane.data.weight.empty = Weight_total - (W_fuel+W_payload);
plane.data.weight.fuel = W_fuel;
plane.data.weight.retardent = W_payload;
end
