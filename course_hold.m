function [phi_c] = course_hold(chi_c, chi, r, t, P)
s = P.S_wing
Cpda        = P.Gamma3*P.C_ell_delta_a+P.Gamma4*P.C_n_delta_a;
a_phi2      = .5*P.rho*P.Va_nominal^2*P.S_wing*P.b*Cpda;
omega_n_phi = (abs(a_phi2)*P.delta_a_max/P.e_phi_max)^.5;
omega_n_chi = omega_n_phi/P.W_chi;
Vg          = P.Va_nominal;

K_p_chi     = 2*P.Zeta_chi*omega_n_chi*Vg/P.gravity;

K_i_chi     = omega_n_chi^2*Vg/P.gravity;

phi_c       = K_p_chi*(chi_c - chi) + K_i_chi*(chi_c-chi)/s;