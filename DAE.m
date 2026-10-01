function [dynamics, path, out] = DAE(s, X, U, P, data)

%% Inputs

% States [t u zc dzc z1 dz1]
t   = X(:,1);
u   = X(:,2);
zc  = X(:,3);
dzc = X(:,4);
z   = X(:,5);
dz  = X(:,6);

% Controls kappa
kappa = U(:,1);

% Parameters
chc = P(:,1);
chr = P(:,2);

% Pull from auxdata
g     = data.g;     % gravity
mc    = data.mc;    % chassis mass
m     = data.m;     % wheel mass
k     = data.k;     % suspension stiffness
kt    = data.kt;    % tire stiffness
R     = data.R;     % tire radius
S     = data.S;     % suspension relaxed length
CD    = data.CD;    % drag coefficient
Af    = data.Af;    % frontal area
rho   = data.rho;   % air density
Pmax  = data.Pmax;  % engine power limit
A     = data.A;     % Bump amplitude
sigma = data.sigma; % Bump shape parameter
W     = data.W;     % Bump width
Sb    = data.Sb;    % Bump location

%% Bump logic

% Locate the contact point and calculate the road height and slope
x_qc       = s - Sb;
[r, dr_ds] = sigmoidBump(x_qc, A, W, sigma, data.lengthscale);
theta      = atan(dr_ds); %dr/ds rate of change of road angle (m/m)

% Computational efficiency
C_theta = cos(theta); S_theta = sin(theta);

%% Clearance

contactEps = data.contactEps;            %for numerical purposes
G = R + z - r;                           %Clearance function
Q = 0.5*(G + sqrt(G.^2 + contactEps^2)); %Give me the positive values of G and 0 otherwise. (tire carcass deformation - compression only)

%% Dynamics

%An SAE sign convention is followed here (positive x is forward, positive z is downward)
%A positive suspension force is in compression

% 2-way dampers
ch = (0.5*(chc + chr) + 0.5*(chc - chr).*tanh(dzc - dz)); %Heave damper stiffness

%Internal forces
Fs = k.*(S - (z - zc)); % Suspension spring force where S is the relaxed suspension length and (z1 - zc) is the current suspension length.
Fc = ch.*(dzc - dz);    % Suspension damper force
Fz = kt.*Q;            % Tire carcass force

%The suspension force is the sum of the spring and damper forces
FS = Fs + Fc;

%Tire normal force is slightly different from the tire carcass force by the cosine of the angle theta
N  = Fz.*C_theta;
Nx = N.*S_theta;

%Tire longitudinal force. Positive slip ratio makes forward force
alp     = zeros(size(s));
[Fx, ~] = Tire_force(alp, kappa, N, data.tir);

%Static normal load when the quarter car is stationary (used in cost function)
N_static = (mc + m)*g/kt;

%External forces (aerodynamic drag)
Fax = 0.5.*rho.*CD.*Af.*u.^2;

%Summing forces.
Fxx = Fx + Nx - Fax;   %Force of tire + push from bump - aerodynamic drag
Fzc = mc.*g - FS;       %weight of chassis (down) - suspension force (up)
Fz  = m.*g + FS - Fz; %weight of wheel (down) + suspension force (down) - tire force (up)

%Equations of motion
udot = Fxx./(mc + m);
ddzc = Fzc./mc;
ddz  = Fz./m;

%% Optimal control details
sdot = u;
Sf   = 1./sdot;

dx_1 = Sf;       %t
dx_2 = Sf.*udot; %u
dx_3 = Sf.*dzc;  %zc
dx_4 = Sf.*ddzc; %dzc
dx_5 = Sf.*dz;   %z
dx_6 = Sf.*ddz;  %dz
dynamics = [dx_1,dx_2,dx_3,dx_4,dx_5,dx_6];

con = (Fx.*u)./Pmax; %engine maximum power constraint
path  = con;

%Variation from static load
dN = N - N_static;

%% Output data structure

out.ddzc = ddzc;
out.ddz  = ddz;
out.dN   = dN;
out.r    = r;
out.N    = N;
out.G    = G;
out.Sf   = Sf;