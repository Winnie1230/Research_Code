clear all; close all; clc;

%% Step1
% Define the symoblic state variable x and time t, and assign q = x(1:m),
% q_dot = x(m+1:n)

syms t real
x = sym('x',[4,1],'real');
q = x(1:2); q_dot = x(3:4);

%% Step2
% Define all the numerical parameters(mass, gravity, geometry, etc) and
% save them in param.mat
load('param.mat');

%% Step3
% Define input tau as a function of t and x

tau = sym('tau',[2,1],'real');
% tau = [10;-10]; % set as constant

%% Step4
% Write down the Lagrangian L = T-V

% ----- kinematic energy -----
% position
p = [lc1*cosd(q(1));...
     lc1*sind(q(1));...
     lc2*cosd(q(2));...
     lc2*sind(q(2));...
     l1*cosd(q(1))+lc3*cosd(CalTheta3(q(1),q(2)));...
     l1*sind(q(1))+lc3*sind(CalTheta3(q(1),q(2)));...
     l2*cosd(q(2))+lc4*cosd(CalTheta4(q(1),q(2)));...
     l2*sind(q(2))+lc4*sind(CalTheta4(q(1),q(2)))];
% velocity 
p_dot = jacobian(p,q) * q_dot;
D = diag([m1,m1,m2,m2,m3,m3,m4,m4]);

% angular
a = [q(1);...
     q(2);...
     CalTheta3(q(1),q(2));...
     CalTheta4(q(1),q(2))];
% angular velocity
a_dot = jacobian(a,q) * q_dot;
I = diag([I1,I2,I3,I4]);

T = 1/2*p_dot'*D*p_dot + 1/2*a_dot'*I*a_dot;

% ----- potential energy -----
y = [lc1*sind(q(1));...
     lc2*sind(q(2));...
     l1*sind(q(1))+lc3*sind(CalTheta3(q(1),q(2)));...
     l2*sind(q(1))+lc4*sind(CalTheta4(q(1),q(2)))];
Mg = [m1;m2;m3;m4];

V = g*Mg'*y;

% ----- Lagrangian -----
L = T - V;

%% Step5
% Calculate all the Jacobians
Lqdot_qdot = jacobian(jacobian(L,q_dot),q_dot);
Lqdot_qdot = simplify(Lqdot_qdot);
Inv_Lqdot_qdot = inv(Lqdot_qdot);
Lq_qdot = jacobian(jacobian(L,q_dot),q);
Lq = jacobian(L,q);
f = simplify([q_dot;...
              Inv_Lqdot_qdot*(tau+Lq'-Lq_qdot*q_dot)]);
% f = simplify([q_dot;...
%               jacobian(jacobian(L,q_dot),q_dot)\(tau+jacobian(L,q)'-jacobian(jacobian(L,q_dot),q)*q_dot)]);


%% Step6
% Save f(t,x) as func_f.m for further usage
matlabFunction(f,'file','func_f','vars',{t,x,tau});