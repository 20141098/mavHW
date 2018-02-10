function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(filename,x_trim,u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input
% add stuff here  

[A,B,C,D] = linmod(filename, x_trim, u_trim);

E1 = [...
    0,0,0,0,1,0,0,0,0,0,0,0;...
    0,0,0,0,0,0,0,0,0,1,0,0;...
    0,0,0,0,0,0,0,0,0,0,0,1;...
    0,0,0,0,0,0,1,0,0,0,0,0;...
    0,0,0,0,0,0,0,0,1,0,0,0];
E2 = [...
    0,1,0,0;...
    0,0,1,0];

A_lat = E1*A*E1';
B_lat = E1*B*E2';


E3 = [...
    0,0,0,1,0,0,0,0,0,0,0,0;...
    0,0,0,0,0,1,0,0,0,0,0,0;...
    0,0,0,0,0,0,0,0,0,0,1,0;...
    0,0,0,0,0,0,0,1,0,0,0,0;...
    0,0,1,0,0,0,0,0,0,0,0,0];
E4 = [...
    1,0,0,0;...
    0,0,0,1];
A_lon = E3*A*E3';
B_lon = E3*B*E4';
































% delta_e_trim = u_trim(1);
% delta_a_trim = u_trim(2);
% delta_r_trim = u_trim(3);
% delta_t_trim = u_trim(4);
% 
% u      = x_trim(4);
% v      = x_trim(5);
% w      = x_trim(6);
% phi    = x_trim(7);
% theta  = x_trim(8);
% psi    = x_trim(9);
% p      = x_trim(10);
% q      = x_trim(11);
% r      = x_trim(12);
% Va_trim     = norm(x_trim(4:6)); % norm of xtrim[4-6]
% B_trim      = atan(v/(u^2+w^2)^.5);
% beta_trim = B_trim;
% 
% gamma = P.Jx*P.Jz - P.Jxz^2;
% gamma1 = P.Jxz*(P.Jx - P.Jy + P.Jz)/gamma;
% gamma2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/gamma;
% gamma3 = P.Jz/gamma;
% gamma4 = P.Jxz/gamma;
% gamma5 = (P.Jz - P.Jx)/P.Jy;
% gamma6 = P.Jxz/P.Jy;
% gamma7 = ((P.Jx-P.Jy)*P.Jx + P.Jxz^2)/gamma;
% gamma8 = P.Jx/gamma;
% Cpp = gamma3*P.C_ell_p + gamma4*P.C_n_p;
% Cpr = gamma3*P.C_ell_r + gamma4*P.C_n_r;
% Cp0 = gamma3*P.C_ell_0 + gamma4*P.C_n_0;
% Cpb = gamma3*P.C_ell_beta + gamma4*P.C_n_beta;
% Cpda = gamma3*P.C_ell_delta_a + gamma4*P.C_n_delta_a;
% Cpdr = gamma3*P.C_ell_delta_r + gamma4*P.C_n_delta_r;
% Cr0 = gamma4*P.C_ell_0 + gamma8*P.C_n_0;
% Crb = gamma4*P.C_ell_beta + gamma8*P.C_n_beta;
% Crp = gamma4*P.C_ell_p + gamma8*P.C_n_p;
% Crr = gamma4*P.C_ell_r + gamma8*P.C_n_r;
% Crda = gamma4*P.C_ell_delta_a + gamma8*P.C_n_delta_a;
% Crdr = gamma4*P.C_ell_delta_r + gamma8*P.C_n_delta_r;
% 
% Yv = P.rho*P.S_wing*P.b*v*(P.C_Y_p*p + P.C_Y_r*r)/(4*P.mass*Va_trim) + P.rho*P.S_wing*v*(P.C_Y_0 + P.C_Y_beta*beta_trim + P.C_Y_delta_a*delta_a_trim + P.C_Y_delta_r*delta_r_trim) + P.rho*P.S_wing*P.C_Y_beta*(u^2+w^2)^.5/(2*P.mass);
% Yp = w + P.rho*Va_trim*P.S_wing*P.b*P.C_Y_p/(4*P.mass);
% Yr = -u + P.rho*Va_trim*P.S_wing*P.b*P.C_Y_r/(4*P.mass);
% Lv = P.rho*P.S_wing*P.b^2*v*(Cpp*p+Cpr*r)/(4*Va_trim) + P.rho*P.S_wing*P.b*v*(Cp0+Cpb*beta_trim+Cpda*delta_a_trim+Cpdr*delta_r_trim) + P.rho*P.S_wing*P.b*Cpb*(u^2+w^2)^.5/2;
% Lp = gamma1*q+P.rho*Va_trim*P.S_wing*P.b^2*Cpp/4;
% Lr = -gamma2*q + P.rho*Va_trim*P.S_wing*P.b^2*Cpr/4;
% Nv = P.rho*P.S_wing*P.b^2*v*(Crp*p+Crr*r)/(4*Va_trim) + P.rho*P.S_wing*P.b*v*(Cr0+Crb*beta_trim+Crda*delta_a_trim+Crdr*delta_r_trim) + P.rho*P.S_wing*P.b*Cpb*(u^2+w^2)^.5/2;
% Np = gamma7*q + P.rho*Va_trim*P.S_wing*P.b^2*Crp/4;
% Nr = -gamma1*q+P.rho*Va_trim*P.S_wing*P.b^2*Crr/4;
% 
% Yda = P.rho*Va_trim^2*P.S_wing*P.C_Y_delta_a/(2*P.mass);
% Ydr = P.rho*Va_trim^2*P.S_wing*P.C_Y_delta_r/(2*P.mass);
% Lda = P.rho*Va_trim^2*P.S_wing*P.b*Cpda/2;
% Ldr = P.rho*Va_trim^2*P.S_wing*P.b*Cpdr/2;
% Nda = P.rho*Va_trim^2*P.S_wing*P.b*Crda/2;
% Ndr = P.rho*Va_trim^2*P.S_wing*P.b*Crdr/2;
% 
% VaCos = Va_trim * cos(B_trim);
% A_lat = [...
%     Yv,         Yp/VaCos,   Yr/VaCos,               P.gravity*cos(theta)*cos(phi)/VaCos,    0;
%     Lv*VaCos,   Lp,         Lr,                     0,                                      0;
%     Nv*VaCos,   Np,         Nr,                     0,                                      0;
%     0,          1,          cos(phi)*tan(theta),    q*cos(phi)*tan(theta),                  0;
%     0,          0,          cos(phi)*sec(theta),    p*cos(phi)*sec(theta),                  0];
% 
% B_lat = [...
%     Yda/VaCos,  Ydr/VaCos;...
%     Lda,        Ldr;...
%     Nda,        Ndr;...
%     0,          0;...
%     0,          0];
% 
% Xu = u*P.rho*P.S_wing*(Cx0+Cxa*a+Cx
% 
% VaCosA = Va_trim*cos(alpha);
% A_lon = [...
%     Xu,         Xw*VaCosA,  Xq,         -P.gravity*cos(theta),         0;...
%     Zy/VaCosA,  Zw,         Zq/VaCosA,  -P.gravity*sin(theta)/VaCosA,  0;...
%     Mu,         Mw*VaCosA,  Mq,         0,                                  0;...
%     0,          0,          1,          0,                                  0;
%     sin(theta),    -VaCosA*cos(theta),    0,  u*cos(theta)+w*sin(theta),  0];
% B_lon = [...
%     Xde,        Xdt;...
%     Zde/VaCosA, 0;...
%     Mde,        0;...
%     0,          0;...
%     0,          0];
