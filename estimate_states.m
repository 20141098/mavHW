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
   persistent phat qhat rhat static_pres_hat diff_pres_hat pnhat pehat;
   persistent chihat Vghat phihat thetahat psihat;
   if t == 0
       static_pres_hat = P.pd0 * P.rho * P.gravity;
       diff_pres_hat = P.Va0^2 * P.rho / 2;
       pnhat = P.pn0;
       pehat = P.pe0;
       chihat = 0;
       Vghat = norm(P.x_trim(4:6));
       phihat = P.phi0;
       thetahat = P.theta0;
       psihat = P.psi0;
   end
   a_lpf_p = 0.9;%good
   a_lpf_q = 0.9;%decent
   a_lpf_r = 0.9;%good
   a_lpf_sp = 0.1;%good
   a_lpf_dp = .1;%good
   phat = y_gyro_x;
   qhat = y_gyro_y;
   rhat = y_gyro_z;
   
   static_pres_hat = LPF(static_pres_hat, y_static_pres, a_lpf_sp);
   hhat = static_pres_hat / (P.rho * P.gravity);

   accel = [y_accel_x;y_accel_y;y_accel_z];
%    Vahat = y_diff_pres/P.rho;

   diff_pres_hat = LPF(diff_pres_hat, y_diff_pres, a_lpf_dp);
   Vahat = sqrt(2*diff_pres_hat/P.rho);
   

   phat = LPF(phat, y_gyro_x, a_lpf_p);
   qhat = LPF(qhat, y_gyro_y, a_lpf_q);
   rhat = LPF(rhat, y_gyro_z, a_lpf_r);
   anglehat = attitudeKalman(phihat, thetahat, accel, phat, qhat, rhat, Vahat, P);
   phihat = anglehat(1);
   thetahat = anglehat(2);
   
   
   diff_pres_hat = LPF(diff_pres_hat, y_diff_pres, a_lpf_dp);
   Vahat = sqrt(2*diff_pres_hat/P.rho);
%    pnhat = LPF(pnhat, y_gps_n, a_lpf_n);
%    pehat = LPF(pehat, y_gps_e, a_lpf_e);
%    chihat = LPF(chihat, y_gps_course, a_lpf_chi);
%    Vghat = LPF(Vghat, y_gps_Vg, a_lpf_Vg);
%    y_wind_n = Vahat*cos(psihat) + wnhat - Vghat*cos(chihat);
%    y_wind_e = Vahat*sin(psihat) + wehat - Vghat*sin(chihat);
   gps = [y_gps_n;y_gps_e;y_gps_Vg;y_gps_course];
   
   gpshat = [pnhat;pehat;Vghat;chihat;psihat];
   gpshat = gpsKalman(gpshat,Vahat, qhat,rhat, phihat, thetahat, gps, P);
   pnhat = gpshat(1);
   pehat = gpshat(2);
   Vghat = gpshat(3);
   chihat = gpshat(4);
   psihat = gpshat(5);
   wnhat = Vghat*cos(chihat) - Vahat*cos(psihat);
   wehat = Vghat*sin(chihat) - Vahat*sin(psihat);
   
   
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

function [xhat] = attitudeKalman(phi, theta, accel, p,q,r,Va, P)
    
    xhat = [phi;theta];
    persistent COV
    Q =[1e-10, 0; 0, 1e-10];
    if isempty(COV)
        COV = [1e-50,0;0,1e-50];
    end
    %model propogation
    N = 10;
    for i = 1:N
        f_xhat = [p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
            q*cos(phi) - r*sin(phi)];
        xhat = xhat + (P.Ts/N).*f_xhat; % model propogation
        phi = xhat(1);
        theta = xhat(2);
        
        df_dx = [q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta),     (q*sin(phi)+r*cos(phi))/cos(theta)^2;
            -q*sin(phi)-r*cos(phi),                             0]; %corrected
        A = df_dx;
        COV = COV + (P.Ts/N)*(A*COV + COV*A' + Q);%checked
    end
    % measurement correction
    if norm(accel) > .8*P.gravity && norm(accel) < 1.2*P.gravity
        phi = xhat(1);
        theta = xhat(2);
        Ci = [0,                            q*Va*cos(theta) + P.gravity*cos(theta);
            -P.gravity*cos(phi)*cos(theta), -r*Va*sin(theta)-p*Va*cos(theta)+P.gravity*sin(phi)*sin(theta);
            P.gravity*sin(phi)*cos(theta),  (q*Va + P.gravity*cos(phi))*sin(theta)]; %checked
        Ri = [P.sigma_accel^2,0,0;
            0,P.sigma_accel^2,0;
            0,0,P.sigma_accel^2];
        Li = COV*Ci' / (Ri + Ci*COV*Ci');
        COV = (1-Li*Ci)*COV;%checked
        H = [q*Va*sin(theta) + P.gravity*sin(theta);
            r*Va*cos(theta) - p*Va*sin(theta) - P.gravity*cos(theta)*sin(phi);
            -q*Va*cos(theta)-P.gravity*cos(theta)*cos(phi)];%checked
        residual = Li*(accel - H);
        residual(1) = sat(residual(1),.0001*pi/180);%,-.01);
        residual(2) = sat(residual(2),.0001*pi/180);%,-.01);
        xhat = xhat +  (residual);
    end
    xhat(1) = wrapAngle(xhat(1));
    xhat(2) = wrapAngle(xhat(2));
end
%Vg good, psi good, 
function xhat = gpsKalman(xhat, Va, q, r, phi, theta, y_gps, P)
    %xhat = pn, pe, Vg, chi, psi
    %u = Va, q, r,phi, theta
    persistent COV;
    COV = diag([1e-10, 1e-10, 1e-10, 1e-10, 1e-10]);
    Q = diag([1e-50, 1e-10, 1e-10, 1e-10, 1e-10]);
    N = 50;
    for i = 1:N
        Vg = xhat(3);
        chi = xhat(4);
        psi = xhat(5);
        wn = Vg*cos(chi) - Va*cos(psi);
        we = Vg*sin(chi) - Va*sin(psi);
        if abs(Vg) > 1
            psidot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
            f_x = [Vg*cos(chi);
                Vg*sin(chi);
                psidot*Va*sin(chi-psi);
                P.gravity*tan(phi)*cos(chi-psi)/Vg;
                psidot];
            xhat = xhat + (P.Ts/N)*f_x;
            Vgdot = -Va*psidot*(-wn*sin(psi)+we*cos(psi))/Vg;
            dVgdotdchi = psidot*Va*cos(chi-psi);
            dVgdotdpsi = -psidot*Va*cos(chi-psi);
            dchidotdVg = -P.gravity*tan(phi)*cos(chi-psi)/Vg^2;
            dchidotdchi = -P.gravity*tan(phi)*sin(chi-psi)/Vg;
            dchidotdpsi = P.gravity*tan(phi)*sin(chi-psi)/Vg;
            df_dx = [...
                0,  0,  cos(chi),       -Vg*sin(chi),   0;
                0,  0,  sin(chi),       Vg*cos(chi),    0;
                0,  0,  -Vgdot/Vg,      dVgdotdchi,     dVgdotdpsi;
                0,  0,  dchidotdVg,    dchidotdchi,    dchidotdpsi;
                0,  0,  0,              0,              0];
            A = df_dx;
            COV = COV + (P.Ts/N)*(A*COV + COV*A' + Q);
        end
    end
    
    pn = xhat(1);
    pe = xhat(2);
    Vg = xhat(3);
    chi = xhat(4);
    psi = xhat(5);
    h_x = [pn;pe;Vg;chi];
    dh_dx = [...
        1,  0,  0,  0,  0;
        0,  1,  0,  0,  0;
        0,  0,  1,  0,  0;
        0,  0,  0,  1,  0];
    C = dh_dx;
    R = diag([P.sigma_n_gps^2, P.sigma_e_gps^2, P.sigma_Vg_gps^2, P.sigma_course_gps^2]);
    L = COV*C'/ (R + C*COV*C');
    COV = (1-L*C)*COV;
    xhat = xhat + L*(y_gps-h_x);
    
    while xhat(4) < -pi
        xhat(4) = xhat(4) + 2*pi;
    end
    while xhat(4) > pi
        xhat(4) = xhat(4) - 2*pi;
    end
    
end

% function [xhat] = gpsKalmanAdvanced(xhat, Va, q, r, phi, theta, P)
%     persistent COV;
%     N = 100;
%     COV =  [...
%         1e-10,  0,      1e-20,  1e-20,  1e-30, 1e-30,   1e-30;...n
%         0,      1e-10,  1e-20,  1e-20,  1e-30, 1e-30,   1e-30;...e
%         0,      0,      1e-10,  0,      0,     0,       0;...vg
%         0,      0,      0,      1e-10,  0,     0,       0;...chi
%         0,      0,      0,      0,      1e-10, 0,       0;...wn
%         0,      0,      0,      0,      0,     1e-10,   0;...we
%         0,      0,      0,      0,      0,     0,       1e-10];%psi
% 
%     for i = 1:N
%         Vg = xhat(3);
%         chi = xhat(4);
%         wn = xhat(5);
%         we = xhat(6);
% 
%         psi = xhat(7);
%         psidot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
%         fx = [...
%             Vg*cos(chi);
%             Vg*sin(chi);
%             ((Va*cos(psi)+wn)*(-Va*psidot*sin(psi))+(Va*sin(psi)+we)*(Va*psidot*cos(psi)))/Vg;
%             P.gravity*tan(phi)*cos(chi-psi)
%             0;
%             0;
%             q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)];
%         xhat = (P.Ts/N)*fx;
%         Vgdot = Va*psidot*(-wn*sin(psi) + we*cos(psi))/Vg;
%         dVgdotdpsi = -psidot*Va*(wn*cos(psi) + we*sin(psi))/Vg;
%         dchidotdVg = -P.gravity*tan(phi)*cos(chi-psi)/Vg^2;
%         dchidotdchi = -P.gravity*tan(phi)*sin(chi-psi)/Vg;
%         dchidotdpsi = P.gravity*tan(phi)*sin(chi-psi)/Vg;
%         dfdx = [...
%             0,  0,  cos(chi),   -Vg*sin(chi),   0,                      0,                  0;...
%             0,  0,  sin(chi),   Vg*cos(chi),    0,                      0,                  0;...
%             0,  0,  -Vgdot/Vg,  0,              -psidot*Va*sin(psi),    psidot*Va*cos(psi), dVgdotdpsi;...
%             0,  0,  dchidotdVg, dchidotdchi,    0,                      0,                  dchidotdpsi;...
%             0,  0,  0,          0,              0,                      0,                  0;...
%             0,  0,  0,          0,              0,                      0,                  0;...
%             0,  0,  0,          0,              0,                      0,                  0];
%         A = dfdx;
%         COV = COV + (P.Ts/N)*(A*COV + COV*A' + Q);
%     end
%     
%     hx = [...
%         pn;...
%         pe;...
%         Vg;...
%         chi;...
%         Va*cos(psi) + wn - Vg*cos(chi);...
%         Va*sin(psi) + we - Vg*sin(chi)];
%     dhdx = [...
%         1,  0,  0,          0,              0,  0,  0;...
%         0,  1,  0,          0,              0,  0,  0;...
%         0,  0,  1,          0,              0,  0,  0;...
%         0,  0,  0,          1,              0,  0,  0;...
%         0,  0,  -cos(chi),  Vg*sin(chi),    1,  0,  -Va*sin(psi);...
%         0,  0,  -sin(chi),  -Vg*cos(chi),   0,  1,  Va*cos(psi)];
%     
%     Ci = dhdx;
%     Li = P*Ci'/ (Ri + Ci*P*Ci');
%     COV = (1 - Li*Ci)*COV;
%     xhat = xhat + Li(y - hx);
% 
% end

function [val] = sat(val, max)
    if val > max
        val = max;
    end
    if val < -max
        val = -max;
    end
end

function [angle] = wrapAngle(angle)
    while(angle > pi)
        angle = angle - 2 * pi;
    end
    while(angle < -pi)
        angle = angle + 2*pi;
    end
end