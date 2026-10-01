%Tire data for a Formula 1 car with the Kelly 2008 model
%Refer to Tire_force.m for the calculations

auxdata.tir.Fz_1    = 2000;
auxdata.tir.Fz_2    = 6000; 
auxdata.tir.mux_1   = 1.75;
auxdata.tir.mux_2   = 1.4;
auxdata.tir.kappa_1 = 0.11;
auxdata.tir.kappa_2 = 0.1;
auxdata.tir.muy_1   = 1.8;
auxdata.tir.muy_2   = 1.45;
auxdata.tir.alp_1   = 0.1571;
auxdata.tir.alp_2   = 0.1396;
auxdata.tir.Qx      = 1.9;
auxdata.tir.Qy      = 1.9;
auxdata.tir.Sx      = pi./2./atan(auxdata.tir.Qx);
auxdata.tir.Sy      = pi./2./atan(auxdata.tir.Qy);