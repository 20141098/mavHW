
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch_hold
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.airspeed_pitch_kp * error;
  
  % integral term
  ui = P.airspeed_pitch_ki * integrator;
  
  % implement PID control
  theta_c = sat(up + ui, P.theta_c_max, -P.theta_c_max);
  
  % implement integrator antiwindup
  if P.airspeed_pitch_ki~=0,
    theta_c_unsat = up + ui;
    k_antiwindup = P.Ts/P.airspeed_pitch_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;

end
