clear all; close all; clc;
% simulation about multivariance normal distribution process noises
%% parameters
% mass-spring-damper system
m = 1; c = 5; k = 6;

dt = 0.01; % sampling time

% input force(sine wave)
Amp = 10;
w = 2*pi; % rad/s

% ----- force type -----
FORCE_TYPE = 'sine';
% FORCE_TYPE = 'single_sine';
% ----------------------

%% system dynamic model(discretized)
% state: position and velocity

% continuous-time state space representation
Ac = [0 1;-k/m -c/m];
Bc = [0;1/m];
Cc = [1 0];
Dc = 0;
sys = ss(Ac,Bc,Cc,Dc);

% ----- discretized system -----
d_sys = c2d(sys,dt);
Ad = d_sys.A;
Bd = d_sys.B;
Cd = d_sys.C;
Dd = d_sys.D;
% ------------------------------

% process noise(uncorrelated)
q_p = 0.001;
q_v = 0.05;
q_pv = 0.005;
V = [q_p,q_pv;q_pv,q_v];

%% measurement model(discretized)
H = [1 0];
% measurement noise(Gaussian distribution) 
r = 0.01;
W = r;

%% Kalman filter
iter = 100;

% -----estimator initialization -----
est_x = [0;0];
initStateVar = 0.01;
est_P = [initStateVar, 0; 0, initStateVar];
% -----------------------------------

% ----- True State -----
x_init = mvnrnd(est_x,est_P)';
true_x = x_init;
% ----------------------

% ----- book-keeping -----
trueStateHistory = zeros(iter+1,2);
kfEstHistory = zeros(iter+1,2);
kfVarHistory = zeros(iter+1,2); % only save diagonal terms
kfGainHistory = zeros(iter+1,2);
measHistory = zeros(iter+1,1);

% inital value
trueStateHistory(1,:) = true_x;
kfEstHistory(1,:) = est_x;
kfVarHistory(1,:) = diag(est_P);
kfGainHistory(1,:) = nan;
measHistory(1,:) = nan;

for i=1:iter
    % True System Dynamics
    uf = Amp*sin(w*(i*dt));
    true_x = Ad*true_x + Bd*uf + mvnrnd([0,0],V)';
    
    % measurement
    z = H*true_x + mvnrnd(0,W)';
    
    % Prior update
    xp = Ad*est_x + Bd*uf;
    Pp = Ad*est_P*Ad' + V;
    
    % measurement update
    K = Pp*H'*inv(H*Pp*H'+W);
    xm = xp + K*(z-H*xp);
    Pm = (eye(2)-K*H)*Pp;
    
    est_x = xm;
    est_P = Pm;
    
    % book-keeping
    trueStateHistory(i+1,:) = true_x;
    kfEstHistory(i+1,:) = est_x;
    kfVarHistory(i+1,:) = diag(est_P);
    kfGainHistory(i+1,:) = K;
    measHistory(i+1,:) = z;
end

%% ode45
% ----- state space representation -----
f = @(t,X)[X(2);1/m*Amp*sin(w*t)-k/m*X(1)-c/m*X(2)]; % sin input
% --------------------------------------

T = 0:dt:dt*iter; % sec
[ts,xs] = ode45(f,T,x_init);

%% plot
var_hull = 3;

figure(1);
subplot(2,1,1);
plot((0:iter)*dt,trueStateHistory(:,1),'k','LineWidth',2); hold on
plot((0:iter)*dt,kfEstHistory(:,1),'b','LineWidth',2); hold on
plot((0:iter)*dt,kfEstHistory(:,1)+var_hull*sqrt(kfVarHistory(:,1)),'g--','LineWidth',2); hold on
h = plot((0:iter)*dt,kfEstHistory(:,1)-var_hull*sqrt(kfVarHistory(:,1)),'g--','LineWidth',2); hold on
plot((0:iter)*dt,measHistory,'r.'); hold on
% plot((0:iter)*dt,xs(:,1),'Color',[0.9290 0.6940 0.1250],'LineWidth',2);

% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('True State','Estimate',['Covariance Hull(',num2str(var_hull),'$\sigma )$'],'measurment','Interpreter','latex');
xlabel('Time Step k','Interpreter','latex');
ylabel('Position x','Interpreter','latex');

subplot(2,1,2);
plot((0:iter)*dt,trueStateHistory(:,2),'k','LineWidth',2); hold on
plot((0:iter)*dt,kfEstHistory(:,2),'b','LineWidth',2); hold on
plot((0:iter)*dt,kfEstHistory(:,2)+var_hull*sqrt(kfVarHistory(:,2)),'g--','LineWidth',2); hold on
plot((0:iter)*dt,kfEstHistory(:,2)-var_hull*sqrt(kfVarHistory(:,2)),'g--','LineWidth',2); hold on
xlabel('Time Step k','Interpreter','latex');
ylabel('Velocity $\dot{x}$','Interpreter','latex');

figure(2);
subplot(2,1,1);
plot((0:iter)*dt,trueStateHistory(:,1)-kfEstHistory(:,1),'b','LineWidth',2); hold on
plot((0:iter)*dt,var_hull*sqrt(kfVarHistory(:,1)),'g--','LineWidth',2); hold on
h = plot((0:iter)*dt,-var_hull*sqrt(kfVarHistory(:,1)),'g--','LineWidth',2); hold on
% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('estimation error',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'Interpreter','latex');
xlabel('Time Step k','Interpreter','latex');
ylabel('Position error','Interpreter','latex');

subplot(2,1,2);
plot((0:iter)*dt,trueStateHistory(:,2)-kfEstHistory(:,2),'b','LineWidth',2); hold on
plot((0:iter)*dt,var_hull*sqrt(kfVarHistory(:,2)),'g--','LineWidth',2); hold on
plot((0:iter)*dt,-var_hull*sqrt(kfVarHistory(:,2)),'g--','LineWidth',2); hold on
xlabel('Time Step k','Interpreter','latex');
ylabel('Velocity error','Interpreter','latex');

