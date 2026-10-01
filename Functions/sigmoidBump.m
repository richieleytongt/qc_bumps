% Sigmoid bump profile function
% x: Longitudinal position (scaled)
% A: Bump height (m) (scaled)
% W: Bump width (m) (scaled)
% sigma: Steepness of the sigmoid
% Lscale: Length scale (m)

function [h, dh] = sigmoidBump(x, A, W, sigma,Lscale)
    x_actual = x./Lscale;

    % Original bump profile (rising edge + falling edge back to 0):
h_actual = -A*0.5.*(tanh(sigma.*(x_actual + W/2)./2) - tanh(sigma.*(x_actual - W/2)./2));
h = h_actual*Lscale;
dh = A.*sigma./4.*(tanh(0.25.*sigma*(2.*x_actual + W)).^2 - tanh(0.25.*sigma.*(-2.*x_actual + W)).^2);

    % Step profile: keep only the rising edge, drop the "back half" so the
    % curb stays at -A instead of returning to 0.
% h_actual = -A*0.5.*(tanh(sigma.*(x_actual + W/2)./2) + 1);
% h = h_actual*Lscale;
% dh = -A.*sigma./4.*(1 - tanh(sigma.*(x_actual + W/2)./2).^2);
end