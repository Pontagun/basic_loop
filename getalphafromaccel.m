function [alpha_acc] = getalphafromaccel(AcceleroXYZ)
N = max(size(AcceleroXYZ));
alpha_acc = zeros(N);
thMTNLNS = 0.25;
w_alpha = .25; % weight of gamma filter
m_g_alpha = 1.2; % weight of linear acceleration equation for gyro.
m_a_alpha = 1.5; % weight of linear acceleration equation for accel.
window = 2;

for c = window:N(1)
    % gyrodiff(c-window+1,:) = max(abs((GyroXYZ(c,:)) - (GyroXYZ(c-window+1, :))));
    %
    % if gyrodiff(c-window+1,:) <= 0.5
    %     gyrodiff(c-window+1,:) = 1 - (gyrodiff(c-window+1,:) / thMTNLNS);
    % else
    %     gyrodiff(c-window+1,:) = 0;
    % end

    acceldiff(c-window+1, :) = max(abs(AcceleroXYZ(c,:) - AcceleroXYZ(c-window+1, :)));

    if acceldiff(c-window+1,:) <= 0.5
        acceldiff(c-window+1,:) = 1 - (acceldiff(c-window+1,:) / thMTNLNS);
    else
        acceldiff(c-window+1,:) = 0;
    end
end

% alpha_diffgyro = 0;
alpha_accelgyro = 0;
% Gamma memory filter
for g = 2:max(size(acceldiff))
    % alpha_diffgyro = (w_alpha * gyrodiff(g-1, :)) + ((1-w_alpha)*alpha_diffgyro);
    alpha_accelgyro = (w_alpha * acceldiff(g-1, :)) + ((1-w_alpha)*alpha_accelgyro);
    % alpha_diffgyros(g-1) = alpha_diffgyro;
    alpha_diffaccel(g-1) = alpha_accelgyro;

end

alpha_acc = alpha_diffaccel;
end