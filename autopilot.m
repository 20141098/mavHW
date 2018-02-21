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
    beta     = uu(6+NN);  % side slip angle
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
    
    autopilot_version = 2;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,beta,Va,Vg,h,chi,phi,theta,p,q,r,t,P);
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
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,beta,Va,Vg,h,chi,phi,theta,p,q,r,t,P)

    mode = 5;
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
            delta_r = 0;%sideslip_hold(beta,t,P);%0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3 % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180;% + h_c;
            chi_c = 0;
            if t==0
                phi_c   = course_hold(chi_c, chi, r, Vg, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, Vg, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
%             delta_t = 1;
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4 % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0
                phi_c   = course_hold(chi_c, chi, r, Vg, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, Vg, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5 % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0
                phi_c   = course_hold(chi_c, chi, r, Vg, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, Vg, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
%             delta_t = 1;
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
    persistent delta_t;
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
            theta_c = 30*pi/180;
            delta_t = .6;%airspeed_with_throttle_hold(Va_c, Va, 1, P);
        case 2  % climb zone
            theta_c = airspeed_with_pitch_hold(Va_c, Va, P);%altitude_hold(h_c, h, P);
            delta_t = .6;%airspeed_with_throttle_hold(Va_c, Va, 1, P);
        case 3 % descend zone
            theta_c = airspeed_with_pitch_hold(Va_c, Va, P);%altitude_hold(h_c, h, P);
            delta_t = 0;%airspeed_with_throttle_hold(Va_c, Va, 1, P);
        case 4 % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, P);
            theta_c = altitude_hold(h_c, h, P)
    end
    
    if h > P.altitude_take_off_zone-5 || altitude_state ~= 1
        if h <= h_c-P.altitude_hold_zone
            altitude_state = 2;
        elseif h >= h_c+P.altitude_hold_zone
            altitude_state = 3;
        else
            altitude_state = 4;
        end
    end
    delta_e = pitch_hold(theta_c, theta, q, P);
%     delta_e = 0;
%     delta_t = 1;
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

%good
function [delta_a] = roll_hold(phi_c, phi, p, P)

    K_p_phi = P.delta_a_max*sign(P.a_phi2)/P.e_phi_max;

    omega_n_phi = (abs(P.a_phi2)*P.delta_a_max/P.e_phi_max)^.5;

    K_d_phi = (2*P.Zeta_phi*omega_n_phi - P.a_phi1)/P.a_phi2;
    
    delta_a = K_p_phi*(phi_c-phi) - K_d_phi*p;
    delta_a = sat(delta_a, 45*pi/180, -45*pi/180);
end
%good but needs tuning
function [phi_c] = course_hold(chi_c, chi, r, flag, P)
    persistent i_error;
    persistent d_error;
    if flag == 1
        i_error = 0;
        d_error = 0;
    end
%     i_error = sat(i_error + (chi_c - chi),pi/4,-pi/4);
    i_error = i_error + (P.Ts/P.K_i_chi) * ((chi_c - chi) + d_error);
    d_error = chi_c - chi;

    phi_c       = P.K_p_chi*(chi_c - chi) + P.K_i_chi*(i_error);
    if P.K_i_chi ~= 0
        i_error = i_error + (P.Ts/P.K_i_chi) * (sat(phi_c, 45*pi/180, -45*pi/180) - phi_c);
    end
    phi_c = sat(phi_c, 45*pi/180, -45*pi/180);
end


function [delta_e] = pitch_hold(theta_c, theta, q, P)
%     theta_c = sat(theta_c, 50*pi/180, -50*pi/180);
    
    delta_e = P.K_p_theta*(theta_c - theta) - P.K_d_theta*q;
    delta_e = sat(delta_e, 45*pi/180, -45*pi/180);
end

function [delta_r] = sideslip_hold(beta, t, P)
    persistent i_error;
    if t == 0;
        i_error = 0;
    end
    K_p_beta = P.delta_r_max*sign(P.a_beta2)/P.e_beta_max;
    K_i_beta = ((P.a_beta1+P.a_beta2*K_p_beta)/(2*P.Zeta_beta))^2/P.a_beta2;
    delta_r = -K_p_beta*beta - K_i_beta*i_error;
end
%good but with overshoot
function [delta_t] = airspeed_with_throttle_hold(Va_c, Va, P)
%     Va_c = Va_c - norm(P.x_trim(4:6));
    persistent i_error;
    if isempty(i_error) == 1
        i_error = 0;
    end
    i_error = i_error + (Va_c - Va);

%     delta_t_trim = P.x_trim(4);

    delta_t = P.K_p_v*(Va_c - Va) + P.K_i_v*i_error;
    delta_t = sat(delta_t, 1, 0);
end
  
function [theta_c] = airspeed_with_pitch_hold(Va_c, Va, P)
    persistent i_error;
    persistent d_error;
    if isempty(i_error) == 1
        i_error = 0;
        d_error = 0;
    end
    i_error = i_error + (Va_c - Va);
    
    theta_c = P.K_p_V2*(Va_c - Va) + P.K_i_V2*i_error;
    if P.K_i_V2 ~= 0
        i_error = i_error + (P.Ts/P.K_i_V2) * (sat(theta_c, 45*pi/180, -45*pi/180) - theta_c);
    end
    theta_c = sat(theta_c, 45*pi/180, -45*pi/180);
end
function [theta_c] = altitude_hold(h_c, h, P)
    persistent i_error;
    persistent d_error;
    if isempty(i_error) == 1
        i_error = 0;
        d_error = 0;
    end
    i_error = i_error + (P.Ts/P.K_i_h) * ((h_c - h) + d_error);
    d_error = h_c - h;

    theta_c = P.K_p_h*(h_c - h);% + P.K_i_h*i_error;
    if P.K_i_h ~= 0
        i_error = i_error + (P.Ts/P.K_i_h) * (sat(theta_c, 30*pi/180, -30*pi/180) - theta_c);
    end
    theta_c = sat(theta_c, 30*pi/180, -30*pi/180);
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
  
 