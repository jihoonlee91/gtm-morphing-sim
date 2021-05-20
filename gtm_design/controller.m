function [sys,x0,str,ts] = controller(t,x,u,flag)

if flag == 1 % dynamics
    ref_dot = zeros(9,1);
    dx = ref_dot;
    sys = dx; % state derivative

elseif flag == 3 % output equation
    % Velocity loop PID gains
    KP1 = 3*eye(3)*1e-1;
    KI1 = eye(3)*1e-4;
    KD1 = eye(3)*1e-5;
    
    % Attitude loop PID gains
    KP2 = 3*eye(3)*1e0;
    KI2 = eye(3)*1e-1;
    KD2 = eye(3)*1e-3;
    
    % Angular velocity loop PID gains
    KP3 = eye(3)*1e1;
    KI3 = eye(3)*1e0;
    KD3 = eye(3)*1e-2;

    % Inputs
    command = u(1:3);
    state = u(4:15);
    actuator = u(16:22);
    acceleration = u(23:25);
    error_int = u(26:34);
    error_dot = u(35:43);
    
    % Velocity commands
    vel_ref = command(1:3);
    
    % States
    VT = state(1); % TAS [m/s]
    alpha = state(2); % AOA [rad]
    beta = state(3); % Sideslip angle [rad]
    p = state(4); % Roll rate [rad/s]
    q = state(5); % Pitch rate [rad/s]
    r = state(6); % Yaw rate [rad/s]
    phi = state(7); % Roll angle [rad]
    theta = state(8); % Pitch angle [rad]
    psi = state(9); % Yaw angle [rad]
    N = state(10); % Northward position [m]
    E = state(11); % Eastward position [m]
    D = state(12); % Downward position [m]
    H = -D; % Altitude [m]
    
    % Actuator
    rpm1 = actuator(1);
    rpm2 = actuator(2);
    rpm3 = actuator(3);
    rpm4 = actuator(4);
    del_e_r = actuator(5);
    del_e_l = actuator(6);
    zeta = actuator(7);
    
    % Body-axis velocities (no wind assumption)
    U = VT*cos(alpha)*cos(beta) ; % x-axis velocity
    V = VT*sin(beta)            ; % y-axis velocity
    W = VT*sin(alpha)*cos(beta) ; % z-axis velocity
    
    % NED velocities
    v_n = U*cos(theta)*cos(psi) ...
        + V*(-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)) ...
        + W*( sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi)) ; % Northward velocity [m/s]
    v_e = U*cos(theta)*sin(psi) ...
        + V*( cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi))...
        + W*(-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi)) ; % Eastward velocity [m/s]
    v_d = -U*sin(theta) ...
        + V*sin(phi)*cos(theta) ...
        + W*cos(phi)*cos(theta) ; % Downward velocity [m/s]
    
    % Flight path angles (no wind assumption)
    gamma = asin(-v_d./VT); % VFPA [rad]
    chi   = atan2(v_e,v_n); % HFPA [rad]
        
    % State vectors
    vel = [VT;gamma;chi];
    omega = [p;q;r]; % Body-axis angular velocity vector [rad/s]
    
    % Velocity tracking errors
    e1 = vel_ref-vel;
    e1_int = error_int(1:3);
    e1_dot = error_dot(1:3);
    
    % Velocity loop PID control
    vel_ref_dot = KP1*e1+KI1*e1_int+KD1*e1_dot;
    VT_ref_dot = vel_ref_dot(1);
    gamma_ref_dot = vel_ref_dot(2);
    chi_ref_dot = vel_ref_dot(3);
    
    % Aerodynamic coefficients
    [Alpha_mesh,ele_mesh] = meshgrid(Alpha_Table,ele_Table);
    CDa = interp1(Alpha_Table,cda_Table,alpha*r2d);
    CD_el_R = 0.5*interp2(Alpha_mesh,ele_mesh,cd_de_Table,alpha*r2d,del_e_r*r2d);
    CD_el_L = 0.5*interp2(Alpha_mesh,ele_mesh,cd_de_Table,alpha*r2d,del_e_l*r2d);
    cyp = interp1(Alpha_Table,cyp_Table,alpha*r2d);
    CLa = interp1(Alpha_Table,CLa_Table,alpha*r2d);
    CL_q = CLq*(q*r2d*c)/(2*VT);
    CL_el_R = 0.5*interp1(ele_Table,CL_de_Table,del_e_r*r2d);
    CL_el_L = 0.5*interp1(ele_Table,CL_de_Table,del_e_l*r2d);
    CD = CD0 + CDa + CD_el_R + CD_el_L;
    CY = cyb_Table*beta*r2d + cyp*(p*r2d*b)/(2*VT);
    CL = CL0 + CLa + CL_q + CL_el_R + CL_el_L;
    
    % Aerodynamic forces
    [~,~,~,rho] = atmosisa(H);
    Q = 0.5*rho*VT^2;
    D = Q*S*CD;
    Y = Q*S*CY;
    L = Q*S*CL;
    X_a = -D*cos(alpha)+L*sin(alpha);
    Y_a = Y;
    Z_a = -D*sin(alpha)-L*cos(alpha);
    
%     VT_dot = ((X_a+20.14*cos(10*pi/180))*cos(alpha)+(Z_a-20.14*sin(10*pi/180))*sin(alpha))/dim.m
%     gamma_dot = ((X_a+20.14*cosd(10))*sin(alpha)-(Z_a-20.14*sind(10))*cos(alpha)-dim.m*Grav)/(dim.m*VT)
    
    % 
    a1 = (dim.m*VT_ref_dot-(X_a*cos(alpha)*cos(beta)+Y_a*sin(beta)+Z_a*sin(alpha)*cos(beta))+dim.m*Grav*sin(gamma))/cos(beta)
    a2 = dim.m*VT*gamma_ref_dot-X_a*sin(alpha)+Z_a*cos(alpha)+dim.m*Grav*cos(gamma)
    T_F_req = 0;
    T1_req = T_F_req/2;
    T2_req = T_F_req-T1_req;
    T_R_req = sqrt(a1^2+a2^2);
    zeta_req = atan2(a2,a1)-alpha;
%     T_R_req = 20.14;
%     zeta_req = 10*pi/180;
    
    % Thrust model inversion
    d = 18;          % [in] Prop. diameter
    prop_pitch = 14; % [in] Prop. pitch
    axial_speed_F = VT*sin(alpha)*cos(beta);
    axial_speed_R = VT*cos(alpha)*cos(beta)*cos(zeta);
    p1 = (1.225*((pi*(0.0254*d)^2)/4)*(((0.0254*prop_pitch/60)^2))*(d/(3.29546*prop_pitch))^1.50);
    p2_F = (1.225*((pi*(0.0254*d)^2)/4)*((0.0254*prop_pitch/60)*axial_speed_F*0.85)*(d/(3.29546*prop_pitch))^1.50);
    p2_R = (1.225*((pi*(0.0254*d)^2)/4)*((0.0254*prop_pitch/60)*axial_speed_R*0.85)*(d/(3.29546*prop_pitch))^1.50);
    rpm1_req = (-p2_F+sqrt(p2_F^2-4*p1*(-T1_req)))/(2*p1);
    rpm2_req = (-p2_F+sqrt(p2_F^2-4*p1*(-T2_req)))/(2*p1);

    % Thrust model
    T1 = p1*rpm1^2+p2_F*rpm1;
    T2 = p1*rpm2^2+p2_F*rpm2;
    T3 = p1*rpm3^2+p2_R*rpm3;
    T4 = p1*rpm4^2+p2_R*rpm4;
    T_F = T1+T2;
    T_R = T3+T4;
    
    % 
%     beta_req = (-dim.m*VT*cos(gamma)*chi_ref_dot+Y_a)/(T_R_req*cos(alpha+zeta_req)-T_F_req*sin(alpha)+X_a*cos(alpha)+Z_a*sin(alpha))
%     (T_R_req*cos(alpha+zeta_req)-T_F_req*sin(alpha)+X_a*cos(alpha)+Z_a*sin(alpha))
    beta_req = 0*pi/180;
    
    % Attitude commands
    phi_ref = 0; % Skid-to-turn constraint
    theta_ref = x_trim(8); % Pitch hold
    beta_ref = beta_req;
%     att_ref = [phi_ref;theta_ref;beta_ref];
    att_ref = [phi_ref;theta_ref;beta_ref];
    
    % Attitude tracking errors
    att = [phi;theta;beta];
    e2 = att_ref-att;
    e2_int = error_int(4:6);
    e2_dot = error_dot(4:6);
    
    % Attitude loop PID control
    att_ref_dot = KP2*e2+KI2*e2_int+KD2*e2_dot;
    
    % Acceleration
    ax = acceleration(1);
    ay = acceleration(2);
    az = acceleration(3);
    A_x = ax-Grav*sin(theta);
    A_y = ay+Grav*cos(theta)*sin(phi);
    A_z = az+Grav*cos(theta)*cos(phi);
    
    % 
    omega_req = pinv([1 sin(phi)*tan(theta) cos(phi)*tan(theta)
        0 cos(phi) -sin(phi)
        W/sqrt(U^2+W^2) 0 -U/sqrt(U^2+W^2)])*(att_ref_dot-...
        [0;
        0;
        1/sqrt(U^2+W^2)*(-U*V/(VT^2)*A_x+(1-(V^2)/(VT^2))*A_y-V*W/(VT^2)*A_z)]);
    
    % Angular velocity commands
    omega_ref = omega_req;
    
    % Angular velocity tracking errors
    e3 = omega_ref-omega;
    e3_int = error_int(7:9);
    e3_dot = error_dot(7:9);
    
    % Attitude loop PID control
    omega_ref_dot = KP3*e3+KI3*e3_int+KD3*e3_dot;
    
    % 
    inertia = [dim.Ix_f dim.Ixy_f dim.Ixz_f;
        dim.Iyx_f dim.Iy_f dim.Iyz_f;
        dim.Izx_f dim.Izy_f dim.Iz_f];
    torque_req = inertia*omega_ref_dot+cross(omega,inertia*omega);
    
    % Aerodynamic moment coefficients
    clb = interp1(Alpha_Table,clb_Table,alpha*r2d)*beta*r2d;
    clp = interp1(Alpha_Table,clp_Table,alpha*r2d)*(p*r2d*b)/(2*VT);
    clr = interp1(Alpha_Table,clr_Table,alpha*r2d)*(r*r2d*b)/(2*VT);
%     Cl_el_R = 0.5*interp1(ele_Table,cl_dail_Table_R,del_e_r*r2d);
%     Cl_el_L = 0.5*interp1(ele_Table,cl_dail_Table_L,del_e_l*r2d);
    cnb = cnb_Table*beta*r2d;
    cnp = interp1(Alpha_Table,cnp_Table,alpha*r2d)*(p*r2d*b)/(2*VT);
    cnr = interp1(Alpha_Table,cnr_Table,alpha*r2d)*(r*r2d*b)/(2*VT);
%     Cn_el_R = 0.5*interp2(Alpha_mesh,ele_mesh,cn_dail_Table_R,alpha*r2d,del_e_r*r2d);
%     Cn_el_L = 0.5*interp2(Alpha_mesh,ele_mesh,cn_dail_Table_L,alpha*r2d,del_e_l*r2d);
    Cma = interp1(Alpha_Table,cm_alpha_Table,alpha*r2d);
    Cmq = cmq*(q*r2d*c)/(2*VT);
%     Cm_el_R = 0.5*interp1(ele_Table,cm_de_Table,del_e_r*r2d);
%     Cm_el_L = 0.5*interp1(ele_Table,cm_de_Table,del_e_l*r2d);
%     Cl = clb+clp+clr+Cl_el_R+Cl_el_L
%     Cm = cm0+Cma+Cmq+Cm_el_R+Cm_el_L
%     Cn = cnb+cnp+cnr+Cn_el_R+Cn_el_L
    Cl_nom = clb+clp+clr;
    Cm_nom = cm0+Cma+Cmq;
    Cn_nom = cnb+cnp+cnr;
    CD_nom = CD0 + CDa;
    CY_nom = CY; 
    CL_nom = CL0 + CLa + CL_q;
    
    % Control derivative
    delta_e = 0.05*pi/180;
    dCl_el_R = (0.5*interp1(ele_Table,cl_dail_Table_R,(del_e_r+delta_e)*r2d)-0.5*interp1(ele_Table,cl_dail_Table_R,(del_e_r-delta_e)*r2d))/(2*delta_e);
    dCl_el_L = (0.5*interp1(ele_Table,cl_dail_Table_L,(del_e_l+delta_e)*r2d)-0.5*interp1(ele_Table,cl_dail_Table_L,(del_e_l-delta_e)*r2d))/(2*delta_e);
    dCm_el_R = (0.5*interp1(ele_Table,cm_de_Table,(del_e_r+delta_e)*r2d)-0.5*interp1(ele_Table,cm_de_Table,(del_e_r-delta_e)*r2d))/(2*delta_e);
    dCm_el_L = (0.5*interp1(ele_Table,cm_de_Table,(del_e_l+delta_e)*r2d)-0.5*interp1(ele_Table,cm_de_Table,(del_e_l-delta_e)*r2d))/(2*delta_e);
    dCn_el_R = (0.5*interp2(Alpha_mesh,ele_mesh,cn_dail_Table_R,alpha*r2d,(del_e_r+delta_e)*r2d)-0.5*interp2(Alpha_mesh,ele_mesh,cn_dail_Table_R,alpha*r2d,(del_e_r-delta_e)*r2d))/(2*delta_e);
    dCn_el_L = (0.5*interp2(Alpha_mesh,ele_mesh,cn_dail_Table_L,alpha*r2d,(del_e_l+delta_e)*r2d)-0.5*interp2(Alpha_mesh,ele_mesh,cn_dail_Table_L,alpha*r2d,(del_e_l-delta_e)*r2d))/(2*delta_e);
    dCD_el_R = (0.5*interp2(Alpha_mesh,ele_mesh,cd_de_Table,alpha*r2d,(del_e_r+delta_e)*r2d)-0.5*interp2(Alpha_mesh,ele_mesh,cd_de_Table,alpha*r2d,(del_e_r-delta_e)*r2d))/(2*delta_e);
    dCD_el_L = (0.5*interp2(Alpha_mesh,ele_mesh,cd_de_Table,alpha*r2d,(del_e_l+delta_e)*r2d)-0.5*interp2(Alpha_mesh,ele_mesh,cd_de_Table,alpha*r2d,(del_e_l-delta_e)*r2d))/(2*delta_e);
    dCL_el_R = (0.5*interp1(ele_Table,CL_de_Table,(del_e_r+delta_e)*r2d)-0.5*interp1(ele_Table,CL_de_Table,(del_e_r-delta_e)*r2d))/(2*delta_e);
    dCL_el_L = (0.5*interp1(ele_Table,CL_de_Table,(del_e_l+delta_e)*r2d)-0.5*interp1(ele_Table,CL_de_Table,(del_e_l-delta_e)*r2d))/(2*delta_e);
    
%     del_e_r
%     del_e_l
%     dCl_el_R
%     dCl_el_L
%     dCm_el_R
%     dCm_el_L
%     dCn_el_R
%     dCn_el_L
%     dCD_el_R
%     dCD_el_L
%     dCL_el_R
%     dCL_el_L

    %
    ac_nom = Q*S*diag([b,c,b])*[Cl_nom;Cm_nom;Cn_nom];
    induced_nom = cross([ARGOS.cgx-ARGOS.aerpx;0;ARGOS.cgz-ARGOS.aerpz],Q*S*...
        [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)]*[-CD_nom;CY_nom;-CL_nom]);
    thrust_nom = [0;-ARGOS.L3*T_R*sin(zeta);0];
    effect_ac = Q*S*diag([b,c,b])*[dCl_el_L-dCl_el_R dCl_el_L+dCl_el_R 0;
        dCm_el_L-dCm_el_R dCm_el_L+dCm_el_R 0
        dCn_el_L-dCn_el_R dCn_el_L+dCn_el_R 0];
    effect_induced = [0 -(ARGOS.cgz-ARGOS.aerpz) 0;
        ARGOS.cgz-ARGOS.aerpz 0 -(ARGOS.cgx-ARGOS.aerpx);
        0 ARGOS.cgx-ARGOS.aerpx 0]*Q*S*...
        [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)]*diag([-1,1,-1])*...
        [dCD_el_L-dCD_el_R dCD_el_L+dCD_el_R 0;
        0 0 0;
        dCL_el_L-dCL_el_R dCL_el_L+dCL_el_R 0];
    Tmax = 81.14;
    effect_thrust = [0 0 ARGOS.L4*Tmax*sin(zeta);
        0 0 0;
        0 0 ARGOS.L4*Tmax*cos(zeta)];
%     [dCD_el_L-dCD_el_R dCD_el_L+dCD_el_R 0;
%         0 0 0;
%         dCL_el_L-dCL_el_R dCL_el_L+dCL_el_R 0]
%     effect_ac
%     effect_induced
%     effect_thrust
    
    %
    nom = ac_nom+induced_nom+thrust_nom;
    effect = effect_ac+effect_induced+effect_thrust;
    delta = pinv(effect)*(torque_req-nom);
    del_a = delta(1);
    del_e = delta(2);
    del_D = delta(3);
    del_el_c = del_e+del_a;
    del_er_c = del_e-del_a;
%     T3_req = (T_R_req+del_D*Tmax)/2;
%     T4_req = (T_R_req-del_D*Tmax)/2;
    T3_req = (20.14+del_D*Tmax)/2;
    T4_req = (20.14-del_D*Tmax)/2;
    rpm3_req = (-p2_R+sqrt(p2_R^2-4*p1*(-T3_req)))/(2*p1);
    rpm4_req = (-p2_R+sqrt(p2_R^2-4*p1*(-T4_req)))/(2*p1);
    
    control = [rpm1_req;rpm2_req;rpm3_req;rpm4_req;del_er_c;del_el_c;zeta_req];
%     control = U_trim;
%     control = [U_trim(1:4);del_er_c;del_el_c;U_trim(7)];
%     control = [rpm1_req;rpm2_req;rpm3_req;rpm4_req;U_trim(5:6);zeta_req];
    error = [e1;e2;e3];
    cmd = zeros(9,1);
    ref = zeros(9,1);
    y = [control;error;cmd;ref];
    sys = y; % output

elseif flag == 0 % initialization
    sizes                = simsizes;
    sizes.NumContStates  = 9;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 10;
    sizes.NumInputs      = 39;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    str = [];
    ts  = [0 0];
    x0 = zeros(9,1); % initial condition

else % nothing
    sys = [ ] ;
end
