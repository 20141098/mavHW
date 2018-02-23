% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    persistent error_n error_e error_h;
    if isempty(error_n)
        error_n = 0;
        error_e = 0;
        error_h = 0;
    end
    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + error_n;
    y_gps_e = pe + error_e; 
    y_gps_h = -pd + error_h;
    
    k_gps = 1/1100;
    error_n = exp(-k_gps*P.Ts_gps)*error_n + .21*randn();
    error_e = exp(-k_gps*P.Ts_gps)*error_e + .21*randn();
    error_h = exp(-k_gps*P.Ts_gps)*error_h + .40*randn();
    
    % construct groundspeed and course measurements
    Vn = Va*cos(psi)+wn;
    Ve = Va*sin(psi)+we;
    Vg = sqrt(Vn^2 + Ve^2);
    sigma_vg = .025;
    sigma_chi = sigma_vg/Vg;
    y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi) + we)^2) + sqrt(sigma_vg)*randn();
    y_gps_course = atan2(Va*sin(psi)+we, Va*cos(psi)+wn) + sqrt(sigma_chi)*randn();

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



