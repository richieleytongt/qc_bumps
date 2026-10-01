%% Quarter car model

%Input the car parameters and send scaled information to auxdata.

%masses and inertias
mc = 170; %chassis sprung mass (kg)
m  = 29;  %wheel unsprung mass (kg)

%geometry
R = 0.1; %tire sidewall unloaded radius (m)
S = 0.1; %suspension relaxed length (m)

%heave stiffnesses
k  = 100; %Heave stiffness (N/mm)
kt = 200; %Tire carcass stiffness (N/mm)
ch = 20;  %damper stiffness (Ns/mm)

%engine (w)
Pmax = 735500/4;

%aerodynamics
CD = 6;   %drag coefficient straight mode
Af = 1.0; %frontal area (m^2)

%% Everything to auxdata
auxdata.mc   = mc*massscale;  
auxdata.m    = m*massscale;         
auxdata.R    = R*lengthscale;
auxdata.S    = S*lengthscale;
auxdata.k    = k*1000*springscale; 
auxdata.kt   = kt*1000*springscale;
auxdata.Pmax = Pmax*powerscale; %maximum engine output
auxdata.CD   = CD;
auxdata.Af   = Af*lengthscale^2;