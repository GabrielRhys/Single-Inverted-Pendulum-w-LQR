clear;
clc;

% Defines the symbolic parameters and coordinates
syms M m l I g F
syms x(t) th(t)

% Defines the state derivatives
dx = diff(x, t);
dth = diff(th, t);

% Defines the movement of the pendulum center of mass in order to
% calculate the Lagrangian
X1 = x + l*sin(th);
Y1 = l*cos(th);

v_X1 = diff(X1, t);
v_Y1 = diff(Y1, t);

% Calculates the kinetic energy T of the cart and the pendulum
T_cart = 0.5 * M * dx^2;
T_pend = 0.5 * m * (v_X1^2 + v_Y1^2) + 0.5 * I * dth^2;
T = T_cart + T_pend;

% Calculates the potential energy V
V = m * g * Y1;

% Calculates the Lagrangian L
L = T - V;

% Computing the Euler-Lagrange equation for x and th
dL_ddth = diff(diff(L, dth), t);
dL_dth  = diff(L, th);

dL_ddx = diff(diff(L, dx), t);
dL_x   = diff(L, x);

Eq1 = dL_ddx - dL_x == F;   % cart: motor force F acts on x
Eq2 = dL_ddth - dL_dth == 0; % pendulum: no direct input

% Small angle approximation to remove sines and cosines and linearize the
% equations
oldTerms = [sin(th), cos(th), dth^2];
newTerms = [th, 1, 0];

Eq1Linear = subs(Eq1, oldTerms, newTerms);
Eq2Linear = subs(Eq2, oldTerms, newTerms);

% Temporary formatting to use the equationsToMatrix function
syms ddx_dummy ddth_dummy;

oldAccel = [diff(x, t, 2), diff(th, t, 2)];
newAccel = [ddx_dummy, ddth_dummy];

Eq1Clean = subs(Eq1Linear, oldAccel, newAccel);
Eq2Clean = subs(Eq2Linear, oldAccel, newAccel);

eqs  = [Eq1Clean, Eq2Clean];
vars = [ddx_dummy, ddth_dummy];

[Mmat, Fmat] = equationsToMatrix(eqs, vars);

accelerations = Mmat \ Fmat;

positions  = [x, th];
velocities = [diff(x, t), diff(th, t)];

A_bottom_left  = jacobian(accelerations, positions);
A_bottom_right = jacobian(accelerations, velocities);

B_bottom = jacobian(accelerations, F);

A_top = [0 0 1 0;
         0 0 0 1];
A_bottom = [A_bottom_left, A_bottom_right];

A_symbolic = [A_top; A_bottom];
B_symbolic = [zeros(2, 1); B_bottom];

states = [x(t), th(t), diff(x(t), t), diff(th(t), t)];
zeros_equilibrium = [0, 0, 0, 0];

A_linear_symbolic = subs(A_symbolic, states, zeros_equilibrium);
B_linear_symbolic = subs(B_symbolic, states, zeros_equilibrium);


% M = cart mass (kg), m = pendulum mass (kg),
% l = pivot to pendulum center of mass (m),
% I = pendulum moment of inertia about its center of mass (kg*m^2)
names  = [M, m, l, I, g];
values = [0.085, 0.032295, 0.043962, 0.000051, 9.81];
A_num = subs(A_linear_symbolic, names, values);
B_num = subs(B_linear_symbolic, names, values);

A = double(A_num);
B = double(B_num);

openEigenvalues = eig(A);   

Q = diag([10, 1200, 1, 10]);
R = 1000;

K = lqr(A, B, Q, R);

closedEigenvalues = eig(A - B*K);  
