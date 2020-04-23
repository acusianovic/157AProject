function cf = getThrustCoefficient(rocket, P_a)

    gam = rocket.prop.gam; % specific heat ratio
    PC = rocket.prop.PC; % chamber pressure
    P_e = rocket.prop.P_e; % exit pressure
    exp = rocket.prop.exp; % expansion ratio
    
    c1 = gam+1;c2 = gam - 1;
    c3 = c1/c2;
    cf = sqrt(2*gam^2/c2*(2/c1)^c3*(1-(P_e/PC)^(c2/gam)))+...
        (P_e-P_a)*exp/PC;
    
end