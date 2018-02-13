function [delta_t] = airspeed_with_throttle_hold(Va_c, Va, t, P)
s = P.S_wing
K_p_v = (2*P.Zeta_v*omega_n_v - a_v1)/a_v2;

K_i_v = omega_n_v^2/a_v2;

delta_t_trim = P.x_trim(4);

delta_t = delta_t_trim + K_p_v*(Va_c - Va) + K_i_v*(Va_c-Va)/s;