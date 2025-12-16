function [A, B] = VehicleStateJacFcn(x, u)
% This function calculates the Jacobian of the state equations
% for the vehicle motion model.
%
% Inputs:
%           x: current state values [X, Y, psi, vx, vy, w]
%           u: current input values [ax, delta]
% Outputs:
%           A: Jacobian of states wrt 'x'
%           B: Jacobian of states wrt 'u'
%
%#codegen

%% Vehicle Parameters
m = 2000;   % Vehicle mass (kg)
J = 4000;   % Moment of inertia (kg.m^2)
lf = 1.4;   % Distance from CG to front axle (m)
lr = 1.6;   % Distance from CG to rear axle (m)
Cf = 12e3;  % Front tire cornering stiffness (N/rad)
Cr = 11e3;  % Rear tire cornering stiffness (N/rad)

%% State Variables
X = x(1);
Y = x(2);
psi = x(3);
vx = x(4);
vy = x(5);
w = x(6);

ax = u(1);
delta = u(2);

%% Slip Angles
betaf = atan2(vy + lf*w, vx) - delta;
betar = atan2(vy - lr*w, vx);

%% Lateral Forces
Fyf = -Cf * betaf * cos(delta);
Fyr = -Cr * betar;

%% Jacobian Matrices Initialization
A = zeros(6,6);
B = zeros(6,2);

%% Compute A Matrix (Jacobian of f w.r.t x)
A(1,3) = -vx*sin(psi) - vy*cos(psi);
A(1,4) = cos(psi);
A(1,5) = -sin(psi);

A(2,3) = vx*cos(psi) - vy*sin(psi);
A(2,4) = sin(psi);
A(2,5) = cos(psi);

A(3,6) = 1;

A(4,5) = w;
A(4,6) = vy;

A(5,4) = -w + (2/m) * (-Cf*(-lf*w - vy)*cos(delta)/(vx^2 + (lf*w + vy)^2) - Cr*(lr*w - vy)/(vx^2 + (-lr*w + vy)^2));
A(5,5) = (2/m) * (-Cf*vx*cos(delta)/(vx^2 + (lf*w + vy)^2) - Cr*vx/(vx^2 + (-lr*w + vy)^2));
A(5,6) = -vx + (2/m) * (-Cf*lf*vx*cos(delta)/(vx^2 + (lf*w + vy)^2) + Cr*lr*vx/(vx^2 + (-lr*w + vy)^2));

A(6,4) = (2/J) * (-Cf*lf*(-lf*w - vy)*cos(delta)/(vx^2 + (lf*w + vy)^2) + Cr*lr*(lr*w - vy)/(vx^2 + (-lr*w + vy)^2));
A(6,5) = (2/J) * (-Cf*lf*vx*cos(delta)/(vx^2 + (lf*w + vy)^2) + Cr*lr*vx/(vx^2 + (-lr*w + vy)^2));
A(6,6) = (2/J) * (-Cf*lf^2*vx*cos(delta)/(vx^2 + (lf*w + vy)^2) - Cr*lr^2*vx/(vx^2 + (-lr*w + vy)^2));

%% Compute B Matrix (Jacobian of f w.r.t u)
B(4,1) = 1;
B(5,2) = (2/m) * (Cf*(-delta + atan2(lf*w + vy, vx))*sin(delta) + Cf*cos(delta));
B(6,2) = (2/J) * (Cf*lf*(-delta + atan2(lf*w + vy, vx))*sin(delta) + Cf*lf*cos(delta));

end
