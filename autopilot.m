function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 1;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,Vg,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
           [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,Vg,h,chi,phi,theta,p,q,r,t,P)

    mode = 2;
    switch mode
        case 1 % tune the roll loop -- tuned
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2 % tune the course loop
            if t==0
                phi_c   = course_hold(chi_c, chi, r, Vg, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, Vg, 0, P);
            end                
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3 % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4 % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5 % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
      end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0
        if h<=P.altitude_take_off_zone   
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    % implement state machine
    switch altitude_state
        case 1  % in take-off zone
            theta_c = 0;%30*pi/180;
            delta_t = 1;
        case 2  % climb zone
            theta_c = 0;%attitude_hold_with_pitch(h_c, h, P);
            delta_t = 1;
        case 3 % descend zone
            theta_c = 0;%attitude_hold_with_pitch(h_c, h, P);
            delta_t = 0;
        case 4 % altitude hold zone
            delta_t = .5;
            theta_c = 0;%attitude_hold_with_pitch(h_c, h, P);
    end
    
    if h > P.altitude_take_off_zone || altitude_state ~= 1
        if h <= h_c-P.altitude_hold_zone
            altitude_state = 2;
        elseif h >= h_c+P.altitude_hold_zone
            altitude_state = 3;
        else
            altitude_state = 4;
        end
    end
    delta_e = pitch_hold(theta_c, theta, q, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_TECS
%   - longitudinal autopilot based on total energy control systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on total energy control
    
    
    delta_e = 0;
    delta_t = 0;
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [delta_a] = roll_hold(phi_c, phi, p, P)
    %Do we recalculate a_phi1, a_phi2 or can we use the values calculated in
    %compute_tf_model
    %This one is bad, K_d_phi and K_i_phi cause errors
%     Zeta_phi = 2;
    
    K_p_phi = P.delta_a_max*sign(P.a_phi2)/P.e_phi_max;

    omega_n_phi = (abs(P.a_phi2)*P.delta_a_max/P.e_phi_max)^.5;

    K_d_phi = (2*P.Zeta_phi*omega_n_phi - P.a_phi1)/P.a_phi2;

%     K_i_phi = -1 * (s*(s^2+(P.a_phi1 + P.a_phi2+K_d_phi)*s + P.a_phi2*K_p_phi))/P.a_phi2;

    delta_a = K_p_phi*(phi_c-phi) - K_d_phi*p;
end

function [delta_e] = pitch_hold(theta_c, theta, q, P)

    K_p_theta = P.delta_e_max*sign(P.a_theta3)/P.e_theta_max;
    P.K_theta_DC = K_p_theta*P.a_theta3/(P.a_theta2 + K_p_theta*P.a_theta3);

    w_n_theta = (P.a_theta2 + P.delta_e_max*abs(P.a_theta3)/P.e_theta_max)^.5;

    K_d_theta = (2*P.Zeta_theta*w_n_theta-P.a_theta1)/P.a_theta3;
    delta_e = K_p_theta*(theta_c - theta) - K_d_theta*q;
end

function [phi_c] = course_hold(chi_c, chi, r, Vg, flag, P)
    persistent i_error;
    if flag == 1
        i_error = 0;
    end
    omega_n_phi = (abs(P.a_phi2)*P.delta_a_max/P.e_phi_max)^.5;
    omega_n_chi = omega_n_phi/P.W_chi;

    i_error = i_error + (chi_c - chi);
    K_p_chi     = 2*P.Zeta_chi*omega_n_chi*Vg/P.gravity;

    K_i_chi     = omega_n_chi^2*Vg/P.gravity;

    phi_c       = K_p_chi*(chi_c - chi) + K_i_chi*(i_error);
end

function [delta_t] = airspeed_with_throttle_hold(Va_c, Va, flag, P)
    persistent i_error;
    if flag == 1
        i_error = 0;
    end
    i_error = i_error + (Va_c - Va);
    K_p_v = (2*P.Zeta_V*P.omega_n_v - P.a_V1)/P.a_V2;

    K_i_v = P.omega_n_v^2/P.a_V2;

%     delta_t_trim = P.x_trim(4);

    delta_t = K_p_v*(Va_c - Va) + K_i_v*i_error;
end
  
function [theta_c] = airspeed_with_pitch_hold(Va_c, Va, flag, P)
    persistent i_error;
    if flag == 1
        i_error = 0;
    end
    w_n_theta = (P.a_theta2 + P.delta_e_max*abs(P.a_theta3)/P.e_theta_max)^.5;
    w_n_V2 = w_n_theta/P.W_V2;
    
    K_i_V2 = -w_n_v2^2/(K_theta_DC*g);
    
    K_p_V2 = (P.a_V1 - 2*P.Zeta_V2*w_n_V2)/(K_theta_DC*g);
    
    theta_c = K_p_V2*(Va_c - Va) + K_i_V2*i_error;
end
function [theta_c] = altitude_hold(h_c, h, flag, P)
    persistent i_error;
    if flag == 1
        i_error = 0;
    end
    i_error = i_error + (h_c - h);
    Va_trim    = norm(P.x_trim(4:6));
    w_n_theta = (P.a_theta2 + P.delta_e_max*abs(P.a_theta3)/P.e_theta_max)^.5;
    w_n_h = w_n_theta/P.W_h;
    
    K_i_h = w_n_h^2/(P.K_theta_DC*Va_trim);
    K_p_h = 2*P.Zeta_h*w_n_h/(P.K_theta_DC * Va_trim);

    theta_c = K_p_h*(h_c - h) + K_i_h*i_error;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 