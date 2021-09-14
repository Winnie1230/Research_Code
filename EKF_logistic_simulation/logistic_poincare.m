clear all; close all; clc;
%% parameters
alpha = 0.07;
mu = 3.9;

%% Dynamic system
logis = @(l)(mu*l*(1-l));
xf = @(x,y)((1-alpha)*logis(x) + alpha*logis(y));
yf = @(x,y)((1-alpha)*logis(y) + alpha*logis(x));

%% Poincare plot
% truncate transient state
iter = 20000;

% initialization
% x = unifrnd(0,1);
% y = unifrnd(0,1);
x = 0.5;
y = 0.5;

% ----- book-keeping -----
xHistory = zeros(iter+1,1);
yHistory = zeros(iter+1,1);

% initial value
xHistory(1,1) = x;
yHistory(1,1) = y;

for i = 1:iter
    x = xf(x,y);
    y = yf(x,y);
    
    % ----- book-keeping -----
    xHistory(i+1,1) = x;
    yHistory(i+1,1) = y;
end

%% plot
trunc = 500;

plot(xHistory(trunc:end,1),yHistory(trunc:end,1),'b*');
axis equal
title('Poincare Map');
xlabel('x');
ylabel('y');

%% function
% noise model
% function u = uniform(low, up)
%     u = unifrnd(low,up);
% end