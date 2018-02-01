% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = 0;%wind(1); % steady wind - North
    w_es    = 0;%wind(2); % steady wind - East
    w_ds    = 0;%wind(3); % steady wind - Down
    u_wg    = 0;%wind(4); % gust along body x-axis
    v_wg    = 0;%wind(5); % gust along body y-axis    
    w_wg    = 0;%wind(6); % gust along body z-axis
    
    % compute wind data in NED
    w_n = w_ns + u_wg;
    w_e = w_es + v_wg;
    w_d = w_ds + w_wg;
    
    % compute air data
    Va = sqrt((u - u_wg)^2 + (v - v_wg)^2 + (w - w_wg)^2);
    alpha = atan((w-w_wg)/(u-u_wg));
    beta = asin((v-v_wg)/Va);
    
    % compute external forces and torques on aircraft
    C_X = -P.C_D_alpha*cos(alpha) + P.C_L_alpha*sin(alpha);
    C_X_q = -P.C_D_q * cos(alpha) + P.C_L_q*sin(alpha);
    C_X_de = -P.C_D_delta_e * cos(alpha) + P.C_L_delta_e * sin(alpha);
    C_Z = -P.C_D_alpha*sin(alpha) - P.C_L_alpha * cos(alpha);
    C_Z_q = -P.C_D_q * sin(alpha) - P.C_L_q*cos(alpha);
    C_Z_de = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e * cos(alpha);
    
    Force(1) =  ...
    -P.mass*P.gravity*sin(theta)...
        + .5*P.rho*Va^2*P.S_wing*(C_X + C_X_q * P.c*q/(2*Va) + C_X_de*delta_e)... 
        + .5*P.rho*P.S_prop*P.c*((P.k_motor*delta_t)^2 - Va^2);
    Force(2) =  ...
        P.mass*P.gravity*cos(theta)*sin(phi)...
        + .5*P.rho*Va^2*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b*p/(2*Va) + P.C_Y_r*P.b*r/(2*Va) + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) = ...
        P.mass*P.gravity*cos(theta)*cos(phi)...
        + .5*P.rho*Va^2*P.S_wing*(C_Z + C_Z_q*P.c*q/(2*Va) + C_Z_de*delta_e);
    
    Torque(1) = .5*P.rho*Va^2*P.S_wing*P.b*(P.C_ell_0+P.C_ell_beta*beta+P.C_ell_p*P.b*p/(2*Va)+P.C_ell_r*P.b*r/(2*Va)+P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r) - P.k_T_P*(P.k_Omega*delta_t)^2;
    Torque(2) = .5*P.rho*Va^2*P.S_wing*P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c*q/(2*Va)+P.C_m_delta_e*delta_e);
    Torque(3) = .5*P.rho*Va^2*P.S_wing*P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b*p/(2*Va) + P.C_n_r*P.b*r/(2*Va)+P.C_n_delta_a*delta_a+P.C_n_delta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end


% function [out] = C_D(a)
%     out = P.C_D_p + (P.C_L_0 + P.C_L_alpha*a)^2/(PI*P.e*A*R)
% end
