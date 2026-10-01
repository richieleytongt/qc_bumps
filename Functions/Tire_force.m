%Tire force calculation using Kelly 2008 model
function [Fx, Fy] = Tire_force(alpha, kappa, N, tire)
    
%Tire data
Fz_1    = tire.Fz_1;
Fz_2    = tire.Fz_2; 
mux_1   = tire.mux_1;
mux_2   = tire.mux_2;
kappa_1 = tire.kappa_1;
kappa_2 = tire.kappa_2;
muy_1   = tire.muy_1;
muy_2   = tire.muy_2;
alpha_1 = tire.alp_1;
alpha_2 = tire.alp_2;
Qx      = tire.Qx;
Qy      = tire.Qy;
Sx      = tire.Sx;
Sy      = tire.Sy;

muxmax   = (N - Fz_1).*(mux_2 - mux_1)./(Fz_2 - Fz_1) + mux_1;
muymax   = (N - Fz_1).*(muy_2 - muy_1)./(Fz_2 - Fz_1) + muy_1;
kappamax = (N - Fz_1).*(kappa_2 - kappa_1)./(Fz_2 - Fz_1) + kappa_1;
alphamax = (N - Fz_1).*(alpha_2 - alpha_1)./(Fz_2 - Fz_1) + alpha_1;
kN       = kappa./kappamax;
alpN     = alpha./alphamax;
rho      = sqrt(kN.^2 + alpN.^2 + 1e-12); % regularization inside sqrt for smooth gradient
mux      = muxmax.*sin(Qx.*atan(Sx.*rho));
muy      = muymax.*sin(Qy.*atan(Sy.*rho));
Fx       = mux.*N.*kN./rho;
Fy       = muy.*N.*alpN./rho;