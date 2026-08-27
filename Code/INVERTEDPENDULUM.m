clear;
clc;

% Defines the state-space variables
syms M m1 m2 L1 L2 l1 l2 I1 I2 g F
syms x(t) th1(t) th2(t)

% Defines the state-space derivatives
dx = diff(x, t);
dth1 = diff(th1, t);
dth2 = diff(th2, t);

ddx = diff(x, t, 2);
ddth1 = diff(th1, t, 2);
ddth2 = diff(th2, t, 2);


%Defines the movement of the cart and both links in order to calculate
%the Lagrangian
XM = x;
YM = 0;

X1 = x + l1*sin(th1);
Y1 = l1*cos(th1);

X2 = x + L1*sin(th1) + l2*sin(th2);
Y2 = L1*cos(th1) + l2*cos(th2);

v_X1 = diff(X1, t);
v_Y1 = diff(Y1, t);
v_X2 = diff(X2, t);
v_Y2 = diff(Y2, t);

%Calculates the kinetic energy T of the cart and the links
T_cart = 0.5 * M * dx^2;
T_link1 = 0.5 * m1 * (v_X1^2 + v_Y1^2) + 0.5*I1*dth1^2;
T_link2 = 0.5 * m2 * (v_X2^2 + v_Y2^2) + 0.5 * I2 * dth2^2;
 
%Calculates total kinetic energy T
T = T_cart + T_link1 + T_link2;

%Calculates total potential energy V
V = m1*g*Y1 + m2*g*Y2;

%Calculates the Lagrangian L
L = T - V;

%Computing the Euler-Lagrange Equation for x, theta1, and theta2
dL_ddth1 = diff(diff(L, dth1), t);
dL_dth1 = diff(L, th1);

dL_ddth2 = diff(diff(L, dth2), t);
dL_dth2 = diff(L, th2);

dL_dx = diff(diff(L, dx), t);
dL_x = diff(L, x);

Eq1 = dL_dx - dL_x == F;
Eq2 = dL_ddth1 - dL_dth1 == 0;
Eq3 = dL_ddth2 - dL_dth2 == 0;

%Small angle approximation to remove sines and cosines and linearize the
%equations
oldTerms = [sin(th1),sin(th2),cos(th1),cos(th2),dth1^2,dth2^2,dth1*dth2];
newTerms = [th1,th2,1,1,0,0,0];

Eq1Linear = subs(Eq1,oldTerms,newTerms);
Eq2Linear = subs(Eq2,oldTerms,newTerms);
Eq3Linear = subs(Eq3,oldTerms,newTerms);

%Temporary formatting to use the equationsToMatrix function
syms ddx_dummy ddth1_dummy ddth2_dummy;

oldAccel = [diff(x,t,2), diff(th1,t,2), diff(th2,t,2)];
newAccel = [ddx_dummy, ddth1_dummy, ddth2_dummy];

Eq1Clean = subs(Eq1Linear, oldAccel, newAccel);
Eq2Clean = subs(Eq2Linear, oldAccel, newAccel);
Eq3Clean = subs(Eq3Linear, oldAccel, newAccel);

eqs = [Eq1Clean,Eq2Clean,Eq3Clean];
vars = [ddx_dummy,ddth1_dummy,ddth2_dummy];

[Mmat, Fmat] = equationsToMatrix(eqs, vars);

accelerations = Mmat \ Fmat;

positions = [x, th1, th2];
velocities = [diff(x,t), diff(th1,t), diff(th2,t)];

A_bottom_left = jacobian(accelerations, positions);
A_bottom_right = jacobian(accelerations, velocities);

B_bottom = jacobian(accelerations, F);

A_top = [0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;];
A_bottom = [A_bottom_left, A_bottom_right];

A_symbolic = [A_top; A_bottom];

B_symbolic = [zeros(3,1); B_bottom];

states = [x(t), th1(t), th2(t), diff(x(t),t), diff(th1(t), t), diff(th2(t), t)];
zeros_equilibrium = [0, 0, 0, 0, 0, 0];

A_linear_symbolic = subs(A_symbolic, states, zeros_equilibrium);
B_linear_symbolic = subs(B_symbolic, states, zeros_equilibrium);

%Substituting actual mass and length values of the physical system
names = [M, m1, m2, L1, L2, l1, l2, I1, I2, g];
values = [0.085, 0.032295, 0.032928, 0.097, 0.3, 0.043962, 0.045373, 0.000051, 0.000054, 9.81];
A_num = subs(A_linear_symbolic, names, values);
B_num = subs(B_linear_symbolic, names, values);

A = double(A_num);
openEigenvalues = eig(A);
B = double(B_num);

Q = diag([10, 100, 1200, 1, 10, 50]);
R = 1000;

K = lqr(A, B, Q, R);

closedEigenvalues = eig(A-B*K);