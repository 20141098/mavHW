function [delta_a] = roll_hold(phi_c, phi, p, P)

%Do we recalculate a_phi1, a_phi2 or can we use the values calculated in
%compute_tf_model
s           = P.S_wing;
Cpp         = P.Gamma3*P.C_ell_p+P.Gamma4*P.C_n_p;
Cpda        = P.Gamma3*P.C_ell_delta_a+P.Gamma4*P.C_n_delta_a;
a_phi1      = -.5*P.rho*P.Va_nominal^2*P.b*P.S_wing*P.b^2*Cpp/(2*P.Va_nominal);
a_phi2      = .5*P.rho*P.Va_nominal^2*P.S_wing*P.b*Cpda;

K_p_phi = P.delta_a_max*sign(a_phi2)/P.e_phi_max;

omega_n_phi = (abs(a_phi2)*P.delta_a_max/P.e_phi_max)^.5;

K_d_phi = (2*P.Zeta_phi*omega_n_phi - a_phi1)/a_phi2;

K_i_phi = -1 * (s*(s^2+(a_phi1 + a_phi2+K_d_phi)*s + a_phi2*K_p_phi))/a_phi2;

delta_a = K_p_phi*(phi_c-phi) + K_i_phi*(phi_c-phi)/s - K_d_phi*p;