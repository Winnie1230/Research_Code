clear all; close all; clc;

%% parameters
mu = 3.9;
alpha = 0.07;
a = 0.6;
b = 0.15;

% noise mean
v = a/2; % process noise mean
w = b/2; % measurement noise mean

% noise variance
Q = a^2/12*eye(2); % process noise
R = b^2/12*eye(2); % measurement noise

%% system dynamic model
% system state is defined as s where s(1): x(k) and s(2): y(k)

logis = @(r)mu*r*(1-r); % logistic function

% ----- prediction model -----
q = @(s)[(1-alpha)*logis(s(1))+(1-v)*alpha*logis(s(2)); ...
         (1-alpha)*logis(s(2))+(1-v)*alpha*logis(s(1))];

% ----- linearize about sm and mean of process noise -----
A = @(sm)[(1-alpha)*mu*(1-2*sm(1))   (1-v)*alpha*mu*(1-2*sm(2));...
          (1-v)*alpha*mu*(1-2*sm(1)) (1-alpha)*mu*(1-2*sm(2)) ];

L = @(sm)[0 -alpha*logis(sm(2)) ; -alpha*logis(sm(1)) 0];

%% measurement model
h = @(sp)[sp(1)*(1-w);sp(2)*(1-w)];

% ----- linearize about sp and mean of meas noise -----
H = [1-w 0 ; 0 1-w];

M = @(sp)[-sp(1) 0 ; 0 -sp(2)];

%% Extended Kalman Filter
iter = 20;

% ----- estimator initialization -----
est_s = [1/2 ; 1/2];
initStateVar = 1/12;
est_P = [initStateVar 0 ; 0 initStateVar];

% ----- True State -----
s_init = [unifrnd(0,1) ; unifrnd(0,1)];
s = s_init;

% ----- book-keeping -----
trueStateHistory = zeros(iter+1,2);
ekfEstHistory = zeros(iter+1,2);
ekfVarHistory = zeros(iter+1,2);
measHistory = zeros(iter+1,2);

% initial value
trueStateHistory(1,:) = s;
ekfEstHistory(1,:) = est_s;
ekfVarHistory(1,:) = diag(est_P);
measHistory(1,:) = nan;

for i = 1:iter
    % ---- True System Dynamic -----
    s = [(1-alpha)*logis(s(1))+(1-unifrnd(0,a))*alpha*logis(s(2));...
         (1-alpha)*logis(s(2))+(1-unifrnd(0,a))*alpha*logis(s(1))];
    
    % ----- measurement -----
    z = [s(1)*(1-unifrnd(0,b));s(2)*(1-unifrnd(0,b))];
    
    % prior update
    sp = q(est_s);
    Pp = A(est_s)*est_P*A(est_s)' + L(est_s)*Q*L(est_s)';
    
    % measurement update
    K = Pp*H'*inv(H*Pp*H' + M(sp)*R*M(sp)');
    sm = sp + K*(z-h(sp));
    Pm = (eye(2)-K*H)*Pp;
    
    % update state
    est_s = sm;
    est_P = Pm;
    
    % book-keeping
    trueStateHistory(i+1,:) = s;
    ekfEstHistory(i+1,:) = est_s;
    ekfVarHistory(i+1,:) = diag(est_P);
    measHistory(i+1,:) = z;
end

%% Plot
dt = 1;

var_hull = 3;
subplot(2,1,1);
plot(dt*(0:iter), trueStateHistory(:,1),'k'); hold on
plot(dt*(0:iter), ekfEstHistory(:,1),'b'); hold on
plot(dt*(0:iter), ekfEstHistory(:,1)+var_hull*sqrt(ekfVarHistory(:,1)),'g'); hold on
h = plot(dt*(0:iter), ekfEstHistory(:,1)-var_hull*sqrt(ekfVarHistory(:,1)),'g'); hold on
plot(dt*(0:iter), measHistory(:,1),'r.');

% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('True State','$\hat{x_m}$',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'measurement','Interpreter','latex');
xlabel('Time step k','Interpreter','latex');
ylabel('$x$','Interpreter','latex');

subplot(2,1,2);
plot(dt*(0:iter), trueStateHistory(:,2),'k'); hold on
plot(dt*(0:iter), ekfEstHistory(:,2),'b'); hold on
plot(dt*(0:iter), ekfEstHistory(:,2)+var_hull*sqrt(ekfVarHistory(:,2)),'g'); hold on
h = plot(dt*(0:iter), ekfEstHistory(:,2)-var_hull*sqrt(ekfVarHistory(:,2)),'g'); hold on
plot(dt*(0:iter), measHistory(:,2),'r.');

% the following line skip the name of the previous plot from the legend
% h.Annotation.LegendInformation.IconDisplayStyle = 'off';
% legend('True State','$\hat{x_m}$',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'measurement','Interpreter','latex');
xlabel('Time step k','Interpreter','latex');
ylabel('$y$','Interpreter','latex');



