function xd = LongitudinalDynamicsAircraft(x, u1, u2)
% Parameters
% Default values
m   = 1;
g   = 9.81;
Iyy = 1;

% Longitudinal States
VT    = x(1);
alpha = x(2);
theta = x(3);
Q     = x(4);

DRAG = u1(1);
LIFT = u1(2);
FT   = u1(3);
M    = u2;

% Initialize vector
xd = zeros(5,1);

xd(1) = (FT*cos(alpha) - DRAG - m*g*sin(theta - alpha))/m;
xd(2) = (-FT*sin(alpha) - LIFT + m*g*cos(theta - alpha) + m*VT*Q)/(m*VT);
xd(3) = Q;
xd(4) = M/Iyy;
% Altitude model
xd(5) = VT*sin(theta - alpha);
end