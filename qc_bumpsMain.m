% Optimal control of a quarter car model with a curb input

%% Preamble
clc, clear
auxdata.eps = 1.0e-4; eps = auxdata.eps;
tic
addpath('Functions'); addpath("Data"); addpath("Results");

%% Define scaling and inputs
g = 9.81; %gravity (m/s^2)
auxdata.g0 = g;
auxdata.l0 = 1;
auxdata.m0 = 186.25;

%Scale factors
auxdata.lengthscale  = 1/auxdata.l0;                 lengthscale  = auxdata.lengthscale;
auxdata.massscale    = 1/auxdata.m0;                 massscale    = auxdata.massscale;
auxdata.timescale    = sqrt(auxdata.g0./auxdata.l0); timescale    = auxdata.timescale;
auxdata.velscale     = lengthscale/timescale;        velscale     = auxdata.velscale;
auxdata.accscale     = lengthscale/timescale^2;      accscale     = auxdata.accscale;
auxdata.forcescale   = massscale*accscale;           forcescale   = auxdata.forcescale;
auxdata.powerscale   = forcescale*velscale;          powerscale   = auxdata.powerscale;
auxdata.inertiascale = massscale*lengthscale^2;      inertiascale = auxdata.inertiascale;
auxdata.springscale  = forcescale/lengthscale;       springscale  = auxdata.springscale;
auxdata.dampscale    = massscale/timescale;          dampscale    = auxdata.dampscale;
auxdata.g            = auxdata.g0*accscale;

%Full car model parameters
run('QC.m');
auxdata.rho = 1.2*massscale/(lengthscale^3);

%Curb parameters
auxdata.A     = 0.05*lengthscale; % height [m]
auxdata.sigma = 10;               % Steepness of the sigmoid
auxdata.W     = 3*lengthscale;    % kerb width [m]
auxdata.Sb    = 55*lengthscale;   % Example bump location (10 [m] times the lengthscale)

%Tire data
%Import scaled tire data
addpath('Data'); run('F12014_tire.m');
auxdata.tir.Fz1 = auxdata.tir.Fz_1*forcescale;
auxdata.tir.Fz2 = auxdata.tir.Fz_2*forcescale;

%Contact smoothing parameter (scaled length units) - single source of truth,
%used by both fc_bumpsContinuous.m and fc_bumpsPlot.m so they stay in sync.
auxdata.contactEps = 1e-4*lengthscale;

%Parameterized track
x = linspace(eps,100,100); %distance

%Performance weights
auxdata.speedWeight   = 1;
auxdata.comfortWeight = eps;
auxdata.tireWeight    = 1;
auxdata.ctrlWeight    = eps;

%Define state scale factors: [t u zc dzc z dz]
auxdata.statescale = [timescale, velscale, lengthscale, velscale, lengthscale, velscale];

%Define control scale factors: [kappa3]
auxdata.controlscale = 1;

%% Build bounds structure

iphase = 1;

%Time to finish 
Tmax = 50*timescale;

%Static deflections of the masses due to gravity
%k1 and kt1 are defined in N/mm in CF1.m, so convert to N/m for SI-consistent deflection.
z0  = -R + (mc + m)*g/(kt*1000); %unsprung mass static deflection
zc0 = -S + mc*g/(k*1000) + z0;

%Static deflection conditions
z0min = z0;
z0max = z0;
zcmin = zc0;
zcmax = zc0;

Xmin = -35; Xmax = -Xmin;

%state bounds: [t u zc dzc z dz]
initialstatelower = [   0,  3, zcmin, -eps, z0min, -eps].*auxdata.statescale;
initialstateupper = [   0,  3, zcmax,  eps, z0max,  eps].*auxdata.statescale;
statelower        = [   0,  3,  Xmin, Xmin,  Xmin, Xmin].*auxdata.statescale;
stateupper        = [Tmax, 50,  Xmax, Xmax,  Xmax, Xmax].*auxdata.statescale;
finalstatelower   = [   0,  3,  Xmin, Xmin,  Xmin, Xmin].*auxdata.statescale;
finalstateupper   = [Tmax, 50,  Xmax, Xmax,  Xmax, Xmax].*auxdata.statescale;

%control bounds: kappa
controllower = 0.0.*auxdata.controlscale;
controlupper = 0.1.*auxdata.controlscale;

bounds.phase(iphase).initialstate.lower = initialstatelower;
bounds.phase(iphase).initialstate.upper = initialstateupper;
bounds.phase(iphase).state.lower        = statelower;
bounds.phase(iphase).state.upper        = stateupper;
bounds.phase(iphase).finalstate.lower   = finalstatelower;
bounds.phase(iphase).finalstate.upper   = finalstateupper;
bounds.phase(iphase).control.lower      = controllower;
bounds.phase(iphase).control.upper      = controlupper;

bounds.phase(iphase).path.lower = -Inf;
bounds.phase(iphase).path.upper =    1;

bounds.phase(iphase).integral.lower    = 0;    %Cost functional lower solution space
bounds.phase(iphase).integral.upper    = 100; %Cost functional upper solution space
bounds.phase(iphase).initialtime.lower = x(1)*lengthscale;
bounds.phase(iphase).initialtime.upper = x(1)*lengthscale;
bounds.phase(iphase).finaltime.lower   = x(end)*lengthscale;
bounds.phase(iphase).finaltime.upper   = x(end)*lengthscale;

%% Build 'guess' structure

load('straight.mat') %check before running

dist                         = output.result.solution.phase.time;
solution.X                   = output.result.solution.phase.state;
solution.U                   = output.result.solution.phase.control;
guess.phase(iphase).time     = dist;
guess.phase(iphase).state    = solution.X;
guess.phase(iphase).control  = solution.U;
guess.phase(iphase).integral = output.result.objective;

%% Parameter patch

parameterscale = [dampscale, dampscale];

parameterlower = [1, 1].*1000.*parameterscale;
parameterupper = [70, 70].*1000.*parameterscale;
bounds.parameter.lower = parameterlower;
bounds.parameter.upper = parameterupper;  

% solution.P                   = output.result.solution.parameter;
% guess.parameter              = solution.P;
guess.parameter(:,1) = ch*1000*dampscale;
guess.parameter(:,2) = ch*1000*dampscale;

%% Build 'setup' structure
setup.name                           = 'qc_bumps';
setup.functions.continuous           = @qc_bumpsContinuous;
setup.functions.endpoint             = @qc_bumpsEndpoint;
setup.auxdata                        = auxdata;
setup.nlp.solver                     = 'ipopt';
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.maxiterations = 1500;
setup.nlp.ipoptoptions.warmstart     = 1;
setup.method                         = 'RPM-Differentiation';
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.derivatives.supplier           = 'adigator';
setup.derivatives.dependencies       = 'sparseNaN';
setup.derivatives.derivativelevel    = 'second';
setup.scales.method                  = 'automatic-hybridUpdate'; 
setup.mesh.method                    = 'hp-PattersonRao';
setup.mesh.tolerance                 = 1.0e-5; % maximum relative error over all the segments
Numsec                               = 300;
setup.mesh.phase.colpoints           = 4.*ones(1,Numsec);
setup.mesh.phase.fraction            = 1./Numsec.*ones(1,Numsec);
setup.mesh.colpointsmin              = 16;
setup.mesh.colpointsmax              = 20;
setup.mesh.maxiterations             = 80; % max refinements
setup.displaylevel                   = 2;

%% run GPOPS
output = gpops2(setup);
save('bump_opt','output'); %rename as appropriate
toc

qc_bumpsPlot('bump_opt.mat');