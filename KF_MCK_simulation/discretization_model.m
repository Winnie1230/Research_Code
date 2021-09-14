clear all; close all; clc;
% compare discretized system by discretization method, shift operator and
% matlab c2d function
%% parameters
m = 1; c = 5; k = 6;
dt = 0.1;
f = 1;

% continuous-time system dynamic model
A = [0 1 ; -k/m -c/m];
B = [0 ; 1/m];
C = [1 0];
D = 0;

%% Discretization model
% matlab c2d
sys = ss(A,B,C,D);
d_sys = c2d(sys,dt);

% discretization method
Ad = [3*exp(-2*dt)-2*exp(-3*dt)  exp(-2*dt)-exp(-3*dt);...
      -6*exp(-2*dt)+6*exp(-3*dt) -2*exp(-2*dt)+3*exp(-3*dt)];
Bd = 1/(6*m)*[-3*exp(-2*dt)+2*exp(-3*dt)+1; ...
              6*exp(-2*dt)-6*exp(-3*dt)];

% shift operator
Ad_s = [1 dt ; -k/m*dt -c/m*dt+1];
Bd_s = [0 ; 1/m*dt];

test = 1;
