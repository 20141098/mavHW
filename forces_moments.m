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
    
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
%     w_ns = 0;
%     w_es = 0;
%     w_ds = 0;
%     u_wg = 0;
%     v_wg = 0;
%     w_wg = 0;
    
    % compute wind data in NED
    Rb2i =[cos(theta)*cos(psi),     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
        cos(theta)*sin(psi),        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
        -sin(theta),                sin(phi)*cos(theta),                                cos(phi)*cos(theta)];
    wind = Rb2i*[u_wg;v_wg;w_wg];
    w_n = w_ns + wind(1);
    w_e = w_es + wind(2);
    w_d = w_ds + wind(3);
    
    velocity = Rb2i'*[w_n;w_e;w_d];
    u_r = u - velocity(1);
    v_r = v - velocity(2);
    w_r = w - velocity(3);
    
    % compute air data
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/Va);
    
    % compute external forces and torques on aircraft
    C_L_alpha = P.C_L_0 + P.C_L_alpha*alpha;
    C_D_alpha = P.C_D_0 + P.C_D_alpha*alpha;
    C_X_alpha = -C_D_alpha*cos(alpha) + C_L_alpha*sin(alpha);
    C_X_q = -P.C_D_q*cos(alpha)+P.C_L_q*sin(alpha);
    C_X_de = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    C_Z_alpha = -C_D_alpha*sin(alpha)-C_L_alpha*cos(alpha);
    C_Z_q = -P.C_D_q*sin(alpha)-P.C_L_q*cos(alpha);
    C_Z_de = -P.C_D_delta_e*sin(alpha)-P.C_L_delta_e*cos(alpha);
    
    Force = [
        -P.mass*P.gravity*sin(theta);...
        P.mass*P.gravity*cos(theta)*sin(phi);...
        P.mass*P.gravity*cos(theta)*cos(phi)]...
     + .5*P.rho*Va^2*P.S_wing*[...
        C_X_alpha + C_X_q*P.c*q/(2*Va) + C_X_de;
        P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b*p/(2*Va) + P.C_Y_r*P.b*r/(2*Va)+P.C_Y_delta_a*delta_a+P.C_Y_delta_r*delta_r;
        C_Z_alpha + C_Z_q*P.c*q/(2*Va) + C_Z_de*delta_e]...
     + .5*P.rho*P.S_prop*P.C_prop*[...
        (P.k_motor*delta_t)^2-Va^2;
        0;
        0];
    Force = Force';
        
    torque = .5*P.rho*Va^2*P.S_wing*[...
        P.b*(P.C_ell_0+P.C_ell_beta*beta+P.C_ell_p*P.b*p/(2*Va)+P.C_ell_r*P.b*r/(2*Va)+P.C_ell_delta_a*delta_a+P.C_ell_delta_r*delta_r);
        P.c*(P.C_m_0 + P.C_m_alpha*alpha+P.C_m_q*P.c*q/(2*Va)+P.C_m_delta_e*delta_e);
        P.b*(P.C_n_0 + P.C_n_beta*beta+P.C_n_p*P.b*p/(2*Va)+P.C_n_r*P.b*r/(2*Va)+P.C_n_delta_a*delta_a+P.C_n_delta_r*delta_r)]...
        + [P.k_T_P*(P.k_Omega*delta_t)^2;0;0];
    Torque = torque';
%     Force(1) =  ...
%     -P.mass*P.gravity*sin(theta)...
%         + .5*P.rho*Va^2*P.S_wing*(C_X + C_X_q * P.c*q/(2*Va) + C_X_de*delta_e)... 
%         + .5*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2 - Va^2);%
%     Force(2) =  ...
%         P.mass*P.gravity*cos(theta)*sin(phi)...
%         + .5*P.rho*Va^2*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b*p/(2*Va) + P.C_Y_r*P.b*r/(2*Va) + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);%
%     Force(3) = ...
%         P.mass*P.gravity*cos(theta)*cos(phi)...
%         + .5*P.rho*Va^2*P.S_wing*(C_Z + C_Z_q*P.c*q/(2*Va) + C_Z_de*delta_e);%
    
%     Torque(1) = .5*P.rho*Va^2*P.S_wing*P.b...
%                 *(P.C_ell_0+P.C_ell_beta*beta...
%                   +P.C_ell_p*P.b*p/(2*Va)...
%                   +P.C_ell_r*P.b*r/(2*Va)...
%                   +P.C_ell_delta_a*delta_a ...
%                   + P.C_ell_delta_r*delta_r) ...
%                 - P.k_T_P*(P.k_Omega*delta_t)^2;%
%     %
%     Torque(2) = .5*P.rho*Va^2*P.S_wing*P.c...
%                 *(P.C_m_0 ...
%                   + P.C_m_alpha*alpha ...
%                   + P.C_m_q*P.c*q/(2*Va)...
%                   + P.C_m_delta_e*delta_e);%
%     Torque(3) = .5*P.rho*Va^2*P.S_wing*P.b...
%                 *(P.C_n_0 ...
%                   + P.C_n_beta*beta ...
%                   + P.C_n_p*P.b*p/(2*Va) ...
%                   + P.C_n_r*P.b*r/(2*Va)...
%                   + P.C_n_delta_a*delta_a...
%                   + P.C_n_delta_r*delta_r);%
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end


% function [out] = C_D(a)
%     out = P.C_D_p + (P.C_L_0 + P.C_L_alpha*a)^2/(PI*P.e*A*R)
% end
