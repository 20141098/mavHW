
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_hold
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
  persistent integrator;
  persistent error_d1;
  persistent hdot;
  persistent hdot_d1;
  persistent h_d1;

  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
      hdot = 0;
      hdot_d1 = 0;
      h_d1 = 0;
  end
 
  % compute the current error
  error = h_c - h;
  
  % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  hdot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*hdot_d1...
      + (2/(2*P.tau+P.Ts))*(h - h_d1);

  % proportional term
  up = P.altitude_kp * error;
  
  % integral term
  ui = P.altitude_ki * integrator;
  
  % derivative gain
  ud = P.altitude_kd * hdot;
  
  % implement PID control
    theta_c = sat(up + ui + ud, P.theta_c_max, -P.theta_c_max);
  
  % implement integrator anti-windup
  if P.altitude_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.altitude_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
  hdot_d1 = hdot;
  h_d1 = h;
end
