function [phi, theta, psi] = quat2eu(x1)
% Reorganizing to match KF book page 164, 176. 
%   Detailed explanation goes here
x = zeros(1, 4);

x(1) = x1(4); % Placing W component first.
x(2:4) = x1(1:3);

phi = atan2(2*(x(3) * x(4) + x(1) * x(2)), 1 - 2*(x(2)^2 + x(3)^2));
theta = -asin(2*(x(2) * x(4) - x(1)*x(3)));
psi = atan2(2*(x(2) * x(3) + x(1)*x(4)), 1-2*(x(3)^2 + x(4)^2));


end