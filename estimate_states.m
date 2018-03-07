% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   persistent phat qhat rhat static_pres_hat diff_pres_hat pnhat pehat accel_x accel_y accel_z chihat Vghat;
   if isempty(phat)
       phat = P.p0;
       qhat = P.q0;
       rhat = P.r0;
       static_pres_hat = P.pd0 * P.rho * P.gravity;
       diff_pres_hat = P.Va0^2 * P.rho / 2;
       pnhat = P.pn0;
       pehat = P.pe0;
       accel_x = 0;
       accel_y = 0;
       accel_z = y_accel_z;
       chihat = 0;
       Vghat = 0;
   end
   a_lpf_p = 0.9;%good
   a_lpf_q = 0.9;%decent
   a_lpf_r = 0.9;%good
   a_lpf_sp = 0.1;%good
   a_lpf_dp = .1;%good
   a_lpf_accelx = .1;%bad
   a_lpf_accely = .1;%bad
   a_lpf_accelz = .1;%bad
   a_lpf_n = .8;%good
   a_lpf_e = .1;%good
   a_lpf_chi = .9;%bad
   a_lpf_Vg = .1; %bad
   
   phat = LPF(phat, y_gyro_x, a_lpf_p);
   qhat = LPF(qhat, y_gyro_y, a_lpf_q);
   rhat = LPF(rhat, y_gyro_z, a_lpf_r);
   
   static_pres_hat = LPF(static_pres_hat, y_static_pres, a_lpf_sp);
   hhat = static_pres_hat / (P.rho * P.gravity);
   
   diff_pres_hat = LPF(diff_pres_hat, y_diff_pres, a_lpf_dp);
   Vahat = sqrt(2*diff_pres_hat/P.rho);
   
   accel_x = LPF(accel_x, y_accel_x, a_lpf_accelx);
   accel_y = LPF(accel_y, y_accel_y, a_lpf_accely);
   accel_z = LPF(accel_z, y_accel_z, a_lpf_accelz);
   
%    phihat = atan(accel_y/accel_z);
%    thetahat = asin(accel_x/P.gravity);
   phihat, thetahat = kalman(phihat, thetahat, p, q, r, Va, 0, 0);
   
   pnhat = LPF(pnhat, y_gps_n, a_lpf_n);
   pehat = LPF(pehat, y_gps_e, a_lpf_e);
   chihat = LPF(chihat, y_gps_course, a_lpf_chi);
   Vghat = LPF(Vghat, y_gps_Vg, a_lpf_Vg);
   
   %not yet implemented
%    pnhat = 1;
%    pehat = 2;
%    Vahat = 0;
%    phihat = 0;
%    thetahat = 0;
   chihat = 0;
   Vghat = 0;
   wnhat = 0;
   wehat = 0;
   psihat = 0;
   
   
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end

function [hat] = LPF(old, u, alpha)
    hat = alpha * old + (1-alpha) * u;
end

function [phi, theta] = kalman(phi, theta, p,q,r,Va,zeta_phi,zeta_theta)
    
    u= [p;q;r;Va];
    zeta = [zeta_phi;zeta_theta];
    eta = [eta_phi; eta_theta];
    xhat = [phi;theta];
    for i = 1:N
%         phi_dot = (p + q*sin(phi)*tan(theta) + r * cos(phi)*tan(theta)) + zeta_phi;
%         theta_dot = (q*cos(phi) - r*sin(phi)) + zeta_theta;
        x_dot = [(p + q*sin(phi)*tan(theta) + r * cos(phi)*tan(theta)) + zeta_phi;
            (q*cos(phi) - r*sin(phi)) + zeta_theta];
%         phi = phi + phi_dot * P.Ts;
%         theta = theta + theta_dot * P.Ts;
        xhat = xhat + P.Ts.*x_dot; %[phi_dot; theta_dot];
        phi = xhat(1);
        theta = xhat(2);
        
        A = [q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta), q*sin(phi)-r*cos(phi)/cos(theta)^2;
            -q*sin(phi)-r*cos(phi), 0];
        P = P + P.Ts*(A*P + P*A' + Q);
    end
    Ci = [0,                            q*Va*cos(theta) + P.gravity*cos(theta);
        -P.gravity*cos(phi)*cos(theta), -r*Va*sin(theta)-p*Va*cos(theta)+P.gravity*sin(phi)*sin(theta);
        P.gravity*sin(phi)*cos(theta),  (q*Va + P.gravity*cos(phi))*sin(theta)];
    Li = P*Ci' / (Ri + Ci*P*Ci');
    P = (1-Li*Ci)*P;
    H = [q*Va*sin(theta) + P.gravity*sin(theta);
        r*Va*cos(theta) - p*Va*sin(theta) - P.gravity*cos(theta)*sin(phi);
        -q*Va*cos(theta)-P.gravity*cos(theta)*cos(phi)];
    xhat = xhat + Li*(yi - H);
    
    phi = xhat(1);
    theta = xhat(2);
end