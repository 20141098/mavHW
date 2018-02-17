function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_V1, a_V2]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% add stuff here
%helper variables

p = x_trim(10);
q = x_trim(11);
r = x_trim(12);

delta_e = u_trim(1);
% delta_a = u_trim(2);
% delta_r = u_trim(3);
delta_t = u_trim(4);

Cpp = P.Gamma3*P.C_ell_p+P.Gamma4*P.C_n_p;
Cpda=P.Gamma3*P.C_ell_delta_a+P.Gamma4*P.C_n_delta_a;
%
Va_trim    = norm(x_trim(4:6)); % norm of xtrim[4-6]
a_phi1     = -.5*P.rho*Va_trim^2*P.b*P.S_wing*P.b^2*Cpp/(2*Va_trim);
a_phi2     = .5*P.rho*Va_trim^2*P.S_wing*P.b*Cpda;
a_theta1   = -P.rho*Va_trim^2*P.c*P.S_wing*P.C_m_q*P.c/(4*P.Jy*Va_trim);
a_theta2   = -P.rho*Va_trim^2*P.c*P.S_wing*P.C_m_alpha/(2*P.Jy);
a_theta3   = P.rho*Va_trim^2*P.c*P.S_wing*P.C_m_delta_e/(2*P.Jy);
% d_theta1   = x_trim(11)*(cos(x_trim(7))-1)-x_trim(12);
% d_theta2   = P.Gamma6*(r^2-p^2)+P.Gamma5*p*r+P.rho*Va_trim^2*P.c*P.S_wing*(P.C_m_0-P.C_m_alpha*P.Gamma-P.C_m_q*P.c*d_theta1/(2*Va_trim))/(2*P.Jy)+d_theta1;
theta_trim = x_trim(8);
 

a_V1       = P.rho*Va_trim*P.S_wing*(P.C_D_0+P.C_D_alpha*P.alpha0+P.C_D_delta_e*delta_e)/P.mass + P.rho*P.S_prop*P.C_prop*Va_trim/P.mass;
a_V2       = P.rho*P.S_prop*P.C_prop*P.k_motor^2*delta_t/P.mass;%%%%%
a_V3       = P.gravity*cos(P.gamma);%%%%%
a_beta1    = -P.rho*Va_trim*P.S_wing*P.C_Y_beta/(2*P.mass);
a_beta2    = P.rho*Va_trim*P.S_wing*P.C_Y_beta/(2*P.mass);

    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

