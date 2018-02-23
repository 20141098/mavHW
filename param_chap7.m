P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = inf;%150;         % desired radius (m) - use (+) for right handed orbit, 
P.gamma = gamma
% autopilot sample rate
P.Ts = 0.01;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

P.Gamma = P.Jx*P.Jz - P.Jxz^2;%
P.Gamma1 = P.Jxz*(P.Jx - P.Jy + P.Jz)/P.Gamma;%
P.Gamma2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.Gamma;%
P.Gamma3 = P.Jz/P.Gamma;%
P.Gamma4 = P.Jxz/P.Gamma;%
P.Gamma5 = (P.Jz - P.Jx)/P.Jy;%
P.Gamma6 = P.Jxz/P.Jy;%
P.Gamma7 = ((P.Jx-P.Jy)*P.Jx + P.Jxz^2)/P.Gamma;%
P.Gamma8 = P.Jx/P.Gamma;%


% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;
Va_trim = norm(x_trim(4:6));

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate
% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r, P.a_phi1, P.a_phi2, P.a_theta1, P.a_theta2, P.a_theta3, P.a_V1, P.a_V2, P.a_beta1, P.a_beta2]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);



%
P.delta_a_max = 45*pi/180;
P.e_phi_max = 60*pi/180;
P.Zeta_phi =1;

P.delta_e_max = 45*pi/180;
P.e_theta_max = 30*pi/180;
P.Zeta_theta = .7;


P.Va_nominal = 10;




P.Zeta_V = 4;
P.omega_n_v = .2;


P.delta_r_max = 45*pi/180;
P.e_beta_max = 10;
P.Zeta_beta = 1

P.altitude_take_off_zone = 20;
P.altitude_hold_zone = 15;



%gains

P.K_p_theta = P.delta_e_max*sign(P.a_theta3)/P.e_theta_max;
K_theta_DC = P.K_p_theta*P.a_theta3/(P.a_theta2 + P.K_p_theta*P.a_theta3);

w_n_theta = (P.a_theta2 + P.delta_e_max*abs(P.a_theta3)/P.e_theta_max)^.5;

P.K_d_theta = (2*P.Zeta_theta*w_n_theta-P.a_theta1)/P.a_theta3;


P.K_p_v = (2*P.Zeta_V*P.omega_n_v - P.a_V1)/P.a_V2;

P.K_i_v = P.omega_n_v^2/P.a_V2;


P.W_chi = 20;
P.Zeta_chi = 1;
omega_n_phi = (abs(P.a_phi2)*P.delta_a_max/P.e_phi_max)^.5;
omega_n_chi = omega_n_phi/P.W_chi;
P.K_p_chi     = 2*P.Zeta_chi*omega_n_chi*Va_trim/P.gravity;
P.K_i_chi     = omega_n_chi^2*Va_trim/P.gravity;

P.Zeta_V2 = .1;
P.W_V2 = 10;
w_n_V2 = w_n_theta/P.W_V2;

P.K_i_V2 = -w_n_V2^2/(K_theta_DC*P.gravity);

P.K_p_V2 = (P.a_V1 - 2*P.Zeta_V2*w_n_V2)/(K_theta_DC*P.gravity);

P.Zeta_h = 5;
P.W_h = 15;

w_n_h = w_n_theta/P.W_h;
P.K_i_h = w_n_h^2/(K_theta_DC*Va_trim);
P.K_p_h = 2*P.Zeta_h*w_n_h/(P.K_theta_DC * Va_trim);



%Sensors
P.Ts_gps = 1;
