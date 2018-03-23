
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_r = coordinated_turn_hold(v, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = -v;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.sideslip_kp * error;
  
  % integral term
  ui = P.sideslip_ki * integrator;
  
  % derivative term
  ud = 0;%-P.sideslip_kd * r;
  
  
  % implement PID control
  theta_r = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % implement integrator antiwindup
  if P.sideslip_ki~=0,
    theta_r_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.sideslip_ki;
    integrator = integrator + k_antiwindup*(theta_r-theta_r_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
 
end

  