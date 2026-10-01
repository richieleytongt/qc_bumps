%Dynamics for an optimal control problem that is solved with GPOPS-II/IPOPT.
function phaseout = qc_bumpsContinuous(input)
distance   = input.phase(1).time;
states     = input.phase(1).state;
controls   = input.phase(1).control;
parameters = input.phase(1).parameter;
dataa      = input.auxdata;

%% Run the differential algebraic equation and unpack the variables for integrand minimzation
[dynamics, path, IntData] = DAE(distance, states, controls, parameters, dataa);
ddzc  = IntData.ddzc;
ddz   = IntData.ddz;
dN    = IntData.dN;
Sf    = IntData.Sf;

%Weights
speedWeight   = dataa.speedWeight;
comfortWeight = dataa.comfortWeight;
tireWeight    = dataa.tireWeight;
ctrlWeight    = dataa.ctrlWeight;

%% Outputs

phaseout.path = path;
phaseout.dynamics = dynamics;

phaseout.integrand(:,1) = ...
    Sf.*(...
    speedWeight + ...
    comfortWeight.*(ddzc.^2 + ddz.^2) + ...
    tireWeight.*(dN.^2) + ...
    ctrlWeight.*(controls(:,1).^2)...
    );
end