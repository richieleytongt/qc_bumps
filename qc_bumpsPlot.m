% Plot script for quarter-car bump results
%Idea is to not do any calculations here

function qc_bumpsPlot(resultsFile)
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesFontSize', 20);
set(groot, 'defaultTextFontSize', 20);
set(groot, 'defaultLegendFontSize', 20);
close all; addpath("Functions"); addpath("Data"); addpath("Results");

data = load(resultsFile);
phase = data.output.result.solution.phase;

s  = phase.time;
X = phase.state;
U = phase.control;
P = data.output.result.solution.parameter;
auxdata = data.output.result.setup.auxdata;

dist = s./auxdata.lengthscale;

%% Extract data

% The states [t u zc dzc z dz]
t   = X(:,1)./auxdata.timescale;
u   = X(:,2)./auxdata.velscale;
zc  = X(:,3)./auxdata.lengthscale;
dzc = X(:,4)./auxdata.velscale;
z   = X(:,5)./auxdata.lengthscale;
dz  = X(:,6)./auxdata.velscale;

% The controls [kappa]
kappa = U(:,1);

% The parameters [chc chr]
chc = P(:,1)./auxdata.dampscale;
chr  = P(:,2)./auxdata.dampscale;

%% Run the DAE and extract PlotData

[~, ~, PlotData] = DAE(s, X, U, P, auxdata); %SCALED dynamics! outputs must be unscaled for plotting
r    = PlotData.r./auxdata.lengthscale;    %road height
ddzc = PlotData.ddzc./auxdata.accscale; %sprung mass vertical acceleration
ddz  = PlotData.ddz./auxdata.accscale;  %unsprung mass vertical acceleration
N    = PlotData.N./auxdata.forcescale;  %contact normal force
G    = PlotData.G./auxdata.lengthscale; %ground clearance

%% Okabe Ito palette
% Okabe-Ito palette
OI.orange     = [230, 159, 0]   / 255;
OI.skyBlue    = [86, 180, 233]  / 255;
OI.blueGreen  = [0, 158, 115]   / 255;
OI.yellow     = [240, 228, 66]  / 255;
OI.blue       = [0, 114, 178]   / 255;
OI.vermillion = [213, 94, 0]    / 255;
OI.purple     = [204, 121, 167] / 255;
Nord.red      = [191, 97, 106] / 255;

%% Outputs and figures

fprintf('Optimal compression damping parameter: %g\n', chc);
fprintf('Optimal rebound damping parameter: %g\n', chr);

integral = data.output.result.objective;
fprintf('Integral solution: %g\n', integral);
fprintf('Finish time: %g (s)\n', t(end));

% Main response figure
figure(1); clf;
subplot(4,1,1); hold on; grid on;
plot(dist, u, 'k-', 'LineWidth', 2);
ylabel('u (m/s)');
title('Quarter-car response');

subplot(4,1,2); hold on; grid on;
plot(dist, zc*1000, 'k-', 'LineWidth', 2);
plot(dist, z*1000, 'r-', 'LineWidth', 2);
plot(dist, r*1000, 'b--', 'LineWidth', 1.2);
ylabel('Disp (mm)');
set(gca, 'YDir', 'reverse'); % Invert y-axis for displacement
legend('z_c', 'z', 'road', 'Location', 'best');

subplot(4,1,3); hold on; grid on;
plot(dist, dzc*1000, 'k-', 'LineWidth', 2);
plot(dist, dz*1000, 'r-', 'LineWidth', 2);
ylabel('Vel (mm/s)');
set(gca, 'YDir', 'reverse'); % Invert y-axis for velocity
legend('dz_c', 'dz', 'Location', 'best');

subplot(4,1,4); hold on; grid on;
plot(dist, ddzc, 'k-', 'LineWidth', 2);
plot(dist, ddz, 'r-', 'LineWidth', 2);
ylabel('Acc $(\mathrm{m\,s^{-2}})$');
set(gca,'YDir','reverse'); % Invert y-axis for acceleration
xlabel('s (m)');
legend('ddz_c', 'ddz', 'Location', 'best');

% Control input figure
figure(2); clf; hold on; grid on;
plot(dist, kappa, 'k-', 'LineWidth', 2);
ylabel('$\kappa$');
xlabel('s (m)');
title('Control input');

% Sprung mass comparison figure: acceleration, velocity, and position over shared s.
figure(3); clf;

ax1 = subplot(3,1,1); hold on; grid on;
plot(dist, zc, 'k-', 'LineWidth', 2);
ylabel('$z_c$ (m)');
xlabel('s (m)');
title('Sprung Mass Dynamics');

ax2 = subplot(3,1,2); hold on; grid on;
plot(dist, dzc, 'k-', 'LineWidth', 2);
ylabel('$\dot{z}_c$ (m/s)');

ax3 = subplot(3,1,3); hold on; grid on;
plot(dist, ddzc, 'k--', 'LineWidth', 2);
ylabel('$\ddot{z}_c$ ($\mathrm{m\,s^{-2}}$)');

linkaxes([ax1, ax2, ax3], 'x');

% Unsprung mass comparison figure: acceleration, velocity, and position over shared s.
figure(4); clf;

bx1 = subplot(3,1,1); hold on; grid on;
plot(dist, z, 'r-', 'LineWidth', 2);
ylabel('$z$ (m)');
xlabel('s (m)');
title('Unsprung Mass Dynamics');

bx2 = subplot(3,1,2); hold on; grid on;
plot(dist, dz, 'r-', 'LineWidth', 2);
ylabel('$\dot{z}$ (m/s)');

bx3 = subplot(3,1,3); hold on; grid on;
plot(dist, ddz, 'r--', 'LineWidth', 2);
ylabel('$\ddot{z}$ $(\mathrm{m\,s^{-2}})$');

linkaxes([bx1, bx2, bx3], 'x');

% Contact quantities figure
figure(5); clf;
subplot(2,1,1); hold on; grid on;
plot(dist, N, 'k-', 'LineWidth', 2);
ylabel('N (N)');
title('Contact force and clearance');

subplot(2,1,2); hold on; grid on;
plot(dist, G*1000, 'b-', 'LineWidth', 2);
ylabel('G (mm)');
xlabel('s (m)');

% For publication
pubFig = figure(6); clf;

picturewidth = 17; % set this parameter and keep it forever
hw_ratio = 0.55; % feel free to play with this ratio, lowered to shorten figure height
set(pubFig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
pos = get(pubFig,'Position');
set(pubFig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

subplot(2,1,1); hold on; grid on;
plot(dist, G*1000,'Color', OI.vermillion, 'LineWidth',2.5)
plot(dist, zc*1000,'k-','LineWidth', 2.5);
plot(dist, z*1000,'k--', 'LineWidth', 2.5);
plot(dist, r*1000,'k:', 'LineWidth', 2.5);
ylabel('Height (mm)');
set(gca, 'YDir', 'reverse'); % Invert y-axis for displacement
legend('clearance','sprung mass', 'unsprung mass', 'road', 'Location', 'best');

subplot(2,1,2); hold on; grid on;
colororder([0 0 0; OI.blue])
yyaxis right
plot(dist, u, 'LineWidth',2.5)
ylabel('Longitudinal velocity, $u$ (m/s)')

yyaxis left
plot(dist,N, 'linewidth',2.5)
ylabel('Tire normal force, $N$ (N)')
xlabel('Elapsed distance, $s$ (m)')


end