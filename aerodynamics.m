
%aerodynamics solver function

% TODO: Implement barrowman equations
function rocket = aerodynamics(rocket)
    wing = rocket.geo.wing;
    h_tail = rocket.geo.h_tail;
    v_tail = rocket.geo.v_tail;
    body = rocket.geo.body;
    weight = rocket.data.weight;
    v_stall = rocket.data.requirements.v_stall; %v_ref set to stall velocity, ft/s
    v_max = rocket.data.requirements.v_max;

    v_ref = linspace(v_stall, v_max);

    air_density = 0.001267;      %slug/ft3, 20,000 ft.
    %air_density = 0.002;      %slug/ft3, 20,000 ft.
    viscosity = 3.324*10^-7;     %slug/ft/s, 20,000 ft.
    
    air_density_ground = 0.00238; %quantities at sea
    viscosity_ground = 3.737*10^-7;
    
    e_tail = 0.3;
    e_wing = 0.85;

    CL = zeros(100,2); % 1st column is wet(retardent), 2nd is dry(no retardent)
    CD = zeros(100,2);
    CD0 = zeros(100,2); % 2nd Column is ground conditions
    CDi = zeros(100,2); 
    D = zeros(100,2);
    L = zeros(100,2);
    RE = zeros(100,2);
    
    
    %prop stuff
    eta = rocket.prop.eta_p; % eta and hp same for all planes so I just used the first one
    hp = rocket.prop.hp;
    thrust = (hp * 550 * eta) ./ v_ref;
    thrust = thrust.';
    rocket.prop.thrust = thrust;
    Difference = zeros(100,2);

    for j = 1:2
        for i = 1:100
            %% overall CLs
            if j == 2
                CL(i,j) = weight.dry / ((0.5*air_density*v_ref(i)^2)*wing.S);    %determines dry (~leg 2) CL total for reference speed
            elseif j == 1
                CL(i,j) = weight.wet/((0.5*air_density*v_ref(i)^2)*wing.S);    %determines wet(leg 1) CL total for reference speed
            
            
                %%
                %**** NEED TO DETERMINE WING AND TAIL CL'S INDEPENDENTLY ****


                %% CD0 (parasite) estimation (from slide 93, drag lecture, MAE 154s material)
                % only calculated once rather than twice
                % computing Cf
                Re = air_density*v_ref(i)*wing.c/viscosity;
                Mach = v_ref(i)/968;                      %v_ref MUST BE IN FT/S
                Cf = 0.455/((log10(Re)^2.58)*(1+0.144*Mach^2)^0.65);
                
                %compute K's
                K_wing = (1 + (0.6/wing.h_t)*(wing.ThR) + 100*(wing.ThR)^4)*...
                        (1.34*(Mach^0.18)*cos(wing.sweep*pi/180)^0.28);
                K_horizontal_tail = (1 + (0.6/h_tail.h_t)*(h_tail.ThR) + 100*(h_tail.ThR)^4)*...
                        (1.34*(Mach^0.18)*cos(h_tail.sweep*pi/180)^0.28);
                K_vertical_tail = (1 + (0.6/v_tail.h_t)*(v_tail.ThR) + 100*(v_tail.ThR)^4)*...
                        (1.34*(Mach^0.18)*cos(v_tail.sweep*pi/180)^0.28);
                f_fuselage = body.L/body.W;
                K_fuselage = (1 + (60/f_fuselage^3) + (f_fuselage/400));
                f_nacelle = rocket.geo.nacelle.L/rocket.geo.nacelle.D;
                K_nacelle = 1 + 0.35/f_nacelle;
            %    Cf(i) = 0.455/(((log10(Re))^2.58));
            %     K_wing = 1+2*(wing.ThR)+60*(wing.ThR)^4;
            %     K_horizontal_tail = 1+2*(h_tail.ThR)+60*(h_tail.ThR)^4;
            %     K_vertical_tail = 1+2*(v_tail.ThR)+60*(v_tail.ThR)^4;
            %     
            %     K_fuselage = 1 + 1.5*(body.D/body.L)^(3/2)+7*(body.D/body.L)^3;

                %Q's
                Q_wing = 1;
                Q_tail = 1.08;
                Q_fuselage = 1;
                Q_nacelle = 1.5; %if mounted less than 1 diameter away, then 1.3
                
                CD0_wing = K_wing*Q_wing*Cf*wing.S_wet/wing.S;
                CD0_h_tail = K_horizontal_tail*Q_tail*Cf*h_tail.S_wet/wing.S;
                CD0_v_tail = K_vertical_tail*Q_tail*Cf*v_tail.S_wet/wing.S;
                CD0_fuselage = K_fuselage*Q_fuselage*Cf*(pi*body.L*body.D)/wing.S;
                CD0_nacelle = K_nacelle*Q_nacelle*Cf*rocket.geo.nacelle.S_wet/wing.S;
                
                CD0(i,1) = CD0_wing + CD0_h_tail + CD0_v_tail + CD0_fuselage + 2*CD0_nacelle;
               % CDi(i) = ((CL_wing^2)/(3.1415*wing.AR*e_wing)) +
               % ((CL_tail^2)/(3.1415*tail.AR*e_tail)); %need to figure out how to
               % solve for CL_wing and CL_tail
               
                %% Ground CD0
                Re_ground = air_density_ground*v_ref(i)*wing.c/viscosity_ground;
                Mach_ground = v_ref(i)/1125;                      %v_ref MUST BE IN FT/S
                Cf_ground = 0.455/((log10(Re_ground)^2.58)*(1+0.144*Mach_ground^2)^0.65);
                
                %compute K's
                K_wing_ground = (1 + (0.6/wing.h_t)*(wing.ThR) + 100*(wing.ThR)^4)*...
                        (1.34*(Mach^0.18)*cos(wing.sweep*pi/180)^0.28);
                K_horizontal_tail_ground = (1 + (0.6/h_tail.h_t)*(h_tail.ThR) + 100*(h_tail.ThR)^4)*...
                        (1.34*(Mach^0.18)*cos(h_tail.sweep*pi/180)^0.28);
                K_vertical_tail_ground = (1 + (0.6/v_tail.h_t)*(v_tail.ThR) + 100*(v_tail.ThR)^4)*...
                        (1.34*(Mach^0.18)*cos(v_tail.sweep*pi/180)^0.28);
                K_fuselage_ground = (1 + (60/f_fuselage^3) + (f_fuselage/400));
                K_nacelle_ground = 1 + 0.35/f_nacelle;
                
                CD0_wing_ground = K_wing_ground*Q_wing*Cf_ground*wing.S_wet/wing.S;
                CD0_h_tail_ground = K_horizontal_tail_ground*Q_tail*Cf_ground*h_tail.S_wet/wing.S;
                CD0_v_tail_ground = K_vertical_tail_ground*Q_tail*Cf_ground*v_tail.S_wet/wing.S;
                CD0_fuselage_ground = K_fuselage_ground*Q_fuselage*Cf_ground*(pi*body.L*body.D)/wing.S;
                CD0_nacelle_ground = K_nacelle_ground*Q_nacelle*Cf_ground*rocket.geo.nacelle.S_wet/wing.S;
                CD0(i,2) = CD0_wing_ground + CD0_h_tail_ground + CD0_v_tail_ground + CD0_fuselage_ground + 2*CD0_nacelle_ground;
            end

           %% induced drag
            CDi(i,j) = ((CL(i,j)^2)/(pi*wing.AR*e_wing));    %induced drag
            CD(i,j) = CDi(i,j) + CD0(i,1);                   %total drag
            %CD(i,j) = CD(i,j);                              % compensation basrd on Datacom results
            D(i,j) = 0.5*air_density*v_ref(i)^2*CD(i,j)*wing.S;   %drag force values for dry mass 
            L(i,j) = 0.5*air_density*v_ref(i)^2*CL(i,j)*wing.S;   %dry mass lift force values
            RE(i,j) = Re;
            
           %% Difference between thrust and drag
            Difference(i,j) = thrust(i) - D(i,j);
        
        end
    end                                                     
    
    %Calculate fuel used for first leg of flight
    weightFuel = rocket.prop.fuel_mass;
    weightPayload = rocket.data.weight.retardent;
    weightFinal2 = rocket.data.weight.empty;
    weightInitial = weightFinal2 + weightPayload + weightFuel;

    rocket.data.aero.CL = CL;
    rocket.data.aero.CD = CD;
    %plane.data.CL_alpha = CL_alpha;
    rocket.data.aero.CDi = CDi;
    rocket.data.aero.CD0 = CD0;
    rocket.data.aero.D = D;

    [~, maxDiff_index] = max(Difference,[],1);
    CruiseDrag = D(maxDiff_index);
    rocket.data.aero.CL_cruise = [CL(maxDiff_index(1),1) CL(maxDiff_index(2),2)];
    rocket.data.aero.CD_cruise = [CD(maxDiff_index(1),1) CD(maxDiff_index(2),2)];
    rocket.data.aero.v_cruise = v_ref(maxDiff_index);
    LD = [L(maxDiff_index(1),1)/CruiseDrag(1), L(maxDiff_index(2),2)/CruiseDrag(2)];
    eta_cruise = eta(maxDiff_index);
    rocket.data.aero.LD = LD;
    rocket.data.aero.D = D;
    rocket.data.aero.Re_cruise = RE(maxDiff_index,:);
    rocket.data.aero.cruiseind = maxDiff_index;
    
    f = @(Wi1,Wf1,Wp,Wf2) (eta_cruise(1)*LD(1)*log(Wi1/Wf1) - eta_cruise(2)*LD(2)*log((Wf1-Wp)/Wf2)); %rearranged Bregeut eq whose zero gives weightFinal1
    fun = @(Wf1) f(weightInitial,Wf1,weightPayload,weightFinal2);
    if isreal(weightInitial)
        weightFinal1 = fzero(fun,weightInitial-(weightFuel/2));
    else
        % not ideal but only way this stops breaking
        weightFinal1 = real(weightInitial);
    end
    rocket.data.weight.fuel_1 = weightInitial - weightFinal1;
    rocket.data.weight.fuel_2 = (weightFinal1-weightPayload)-weightFinal2;

    if isreal(CD) && isreal(CL)
        rocket.data.aero.isreal = true;
    else
        % fprintf('imaginary CD or CL for plane \n')
        rocket.data.aero.isreal = false;
    end

end

