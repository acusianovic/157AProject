function rocket = stability(rocket)

    % static stability with wet mass (Cp-Cg)/D
    % static stability with dry mass (Cp-Cg)/D
    % 

    %% static stability with neutral point
    h_acw = rocket.geo.wing.h_ac;
    h_act = rocket.geo.h_tail.h_ac;
    
    S_w = rocket.geo.wing.S;
    S_t = rocket.geo.h_tail.S;
    
    alpha_w = rocket.geo.wing.cl_a;
    alpha_t = rocket.geo.h_tail.cl_a;
    
    epsilon_alpha = 0.3; % random guess, need to refine
    
    h_n = (h_acw + h_act*(S_t/S_w)*(alpha_t/alpha_w) * (1-epsilon_alpha)) / (1 + ((S_t/S_w)*(alpha_t/alpha_w)*(1-epsilon_alpha)));
    
    rocket.data.stability.h_n = h_n;
    h_cg = rocket.geo.wing.h_cg; %h_cg(1) = takeoff cg, h_cg(2) = predrop cg, h_cg(3) = postdrop cg
     
%     l_t = rocket.geo.h_tail.LE - rocket.geo.wing.LE; 
%     c = rocket.geo.wing.c;
%     V_t = l_t*S_t/(S_w*c);
%     h = (rocket.data.weight.CG-(rocket.geo.wing.ac+rocket.geo.wing.LE)) - 0.65*c*V_t;
     
    if all(h_n > h_cg)
        rocket.data.stability.is_stable = true;
    else
        rocket.data.stability.is_stable = false;
    end
    
    %% tail incidence and alpha to trim check
    i_t = zeros(1,5);
    alpha = zeros(1,5);
    wing_c = rocket.geo.wing.c;
    weight = rocket.data.weight;
    
    v_stall = rocket.data.requirements.v_stall;
    v_max = rocket.data.requirements.v_max;
    v_ref = linspace(v_stall, v_max);
    
    alt = 2;
    for i = 1:5
        switch i
            case 1
                W = weight.wet;
                cg_index = 1;
            case 2
                W = weight.wet - weight.fuel_1;
                cg_index = 2;
            case 3
                W = weight.dry - weight.fuel_1;
                cg_index = 3;
            case 4
                W = weight.wet;
                cg_index = 1;
                v_ref = v_stall;
                alt = 1;
            case 5
                W = weight.dry - weight.fuel_1;
                cg_index = 3;
                v_ref = v_stall;
                alt = 1;
        end
        
        [CL, CM_ac, rho, v] = getAero(rocket,W,alt,v_ref);
        l_t = wing_c*(rocket.geo.wing.c*rocket.geo.h_tail.h_ac  - h_cg(cg_index));
        CL_alpha = (alpha_w + alpha_t*(S_t/S_w)*(1 - epsilon_alpha));
        CM_alpha = -CL_alpha*(h_n - h_cg(cg_index));
        CM_i = alpha_t*(l_t*S_t/(wing_c*S_w));
        CL_i = -alpha_t*S_t/S_w;

        i_t(i) = -(CM_ac*CL_alpha + CM_alpha*CL) / (CL_alpha*CM_i - CM_alpha*CL_i); %incidence angle

        % computing alphas
        q = 0.5*rho*v^2;
        alpha(i) = ((W/(q*S_w)) + (alpha_t*i_t(i)*S_t/S_w)) / (alpha_w + (alpha_t*(S_t/S_w)*(1-epsilon_alpha)));
    end
    
    alpha = rad2deg(alpha);
    i_t = rad2deg(i_t);
    
    stall = false;
    if any(abs(alpha) > 15) || any(abs(alpha - i_t) > 15)
        stall = true;
    end
   rocket.data.stability.stall = stall;
   rocket.data.stability.alpha  = alpha; 
   rocket.data.stability.i_t = i_t;
   
   %% yaw stability
    
    rocket.data.stability.yaw_is_stable = true;
    
    for i = 1:2
        b = rocket.geo.wing.b;
        S_w = rocket.geo.wing.S;
        S_v = rocket.geo.v_tail.S;
        v_tail_c = rocket.geo.v_tail.c;
        v_tail_h_cg = rocket.geo.v_tail.h_cg(i);
        l_v = v_tail_h_cg*b - 0.25*v_tail_c;
        L_body = rocket.geo.body.L;
        W_body = rocket.geo.body.W;
        a_v = rocket.geo.v_tail.cl_a;

        V_v = l_v*S_v /(S_w*b);
        V_fuselage = L_body*W_body;

        N_v = 0.8; %vertical tail effectiveness, Q_v/Q
        d_sigma_d_beta = 0.33; %sidewash
        C_y_b_tail = -N_v*V_v*a_v*(1+d_sigma_d_beta);

        C_nb = -C_y_b_tail - 2*V_fuselage/(S_w*b);
        C_nr = 2*C_y_b_tail*(l_v/b)^2;
        C_yr = -2*C_y_b_tail*l_v/b;

        if (C_nb < 0) && (C_nr > 0) && (C_yr < 0)
            rocket.data.stability.yaw_is_stable = false;
        end
        
        rocket.data.aero.C_yb = C_y_b_tail;
        
    end

   
    
end

