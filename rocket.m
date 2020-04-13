function rocket = rocket()
%Creates an instance of the plane class

rocket = struct('geo',[],'prop',[],'data',[]);

%% Aircraft Geometry
rocket.geo = struct('body',[],'wing',[],'h_tail',[],'v_tail',[],'nacelle',[]);
    rocket.geo.body = struct('L',[],'W',[],'D',[]);
    rocket.geo.wing = struct('S',[],'S_wet',[],'AR',[],'c',[],'b',[],'ThR',[],'TR',[],'sweep',[],'cg',[],'ac',[],'h_cg',[],'h_ac',[],'h_t',[],'cl_a',[],'cl_0',[]);
    rocket.geo.h_tail = struct('S',[],'S_wet',[],'AR',[],'c',[],'b',[],'ThR',[],'TR',[],'sweep',[],'cg',[],'ac',[],'h_cg',[],'h_ac',[],'h_t',[],'cl_a',[],'cl_0',[],'i',[]);
    rocket.geo.v_tail = struct('S',[],'S_wet',[],'AR',[],'c',[],'b',[],'ThR',[],'TR',[],'sweep',[],'cg',[],'ac',[],'h_cg',[],'h_ac',[],'h_t',[],'cl_a',[]);
    rocket.geo.nacelle = struct(); % *fill out later*

%% Propulsion
rocket.prop = struct('W',[],'hp',[],'eta_p',[],'c_p',[],'fuel_mass',[]);


%% Performance, Aerodynamics, Stability
rocket.data = struct('requirements',[],'performance',[],'aero',[],'stability',[]);
    rocket.data.requirements = struct('R',500,'v_max',500,'v_stall',146);
    rocket.data.weight = struct('CG',[],'W',[],'x',[],'dry',[],'wet',[],'empty',[],'fuel',[],'fuel_1',[],'retardent',[]);
    rocket.data.performance = struct('R',[],'E',[],'ROC',[],'v_max',366.667,'v_stall',146.667,'N',5);
    rocket.data.aero = struct('CL',[],'CL_alpha',[],'CD',[],'CD0',[],'CDi',[],'v_cruise',[],'LD',[],'CL_cruise',[]);
    rocket.data.stability = struct('h_n',[],'is_stable',[],'yaw_is_stable',[],'stall',[],'alphas',[]);
   
