clear all; close all; clc;
%% parameters
% mass-spring-damper system
m = 1; c = 5; k = 6;

dt = 0.01; % sampling time

% input force(sine wave)
Amp = 10;
w = 2*pi; % rad/s

% ----- noise type -----
% NOISE_TYPE = 'uni';
% NOISE_TYPE = 'nor';
NOISE_TYPE = 'bin';
% ----------------------

% ----- force type -----
FORCE_TYPE = 'sine';
% FORCE_TYPE = 'single_sine';
% ----------------------

%% system dynamic model(discretized)
% without process noise
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
V = [q_p,0;0,q_v];

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

% ---- True State -----
x_init = [noiseModel(NOISE_TYPE,0,initStateVar); noiseModel(NOISE_TYPE,0,initStateVar)];
x = x_init;
% ---------------------

% ----- book-keeping -----
trueStateHistory = zeros(iter+1,2); % true state
kfEstHistory = zeros(iter+1,2);     % state estimate
kfVarHistory = zeros(iter+1,2);     % state variance(keep only the diagonal term)
kfGainHistory = zeros(iter+1,2);    % kalman gain
measHistory = zeros(iter+1,1);      % measurement
inputHistory = zeros(iter+1,1);     % input

% init value
trueStateHistory(1,:) = x;
kfEstHistory(1,:) = est_x;
kfVarHistory(1,:) = diag(est_P);
kfGainHistory(1,:) = nan;
measHistory(1,:) = nan;
inputHistory(1,:) = Force(FORCE_TYPE,0,w,Amp);

for i = 1:iter
    % True system dynamics
    uf = Force(FORCE_TYPE,i*dt,w,Amp); % desire force
    vf_p = noiseModel(NOISE_TYPE,0,q_p);   % position noise
    vf_v = noiseModel(NOISE_TYPE,0,q_v);   % position noise
    x = Ad*x + Bd*uf + [vf_p;vf_v];
    
    % measurement
    z = H*x + noiseModel(NOISE_TYPE,0,r);
    
    % prior update
    xp = Ad*est_x + Bd*uf;
    Pp = Ad*est_P*Ad' + V;
    
    % measurement update
    K = Pp*H'*inv(H*Pp*H'+W);
    xm = xp+K*(z-H*xp);
    Pm = (eye(2)-K*H)*Pp*(eye(2)-K*H)'+K*W*K';
    % Pm = inv(inv(Pp)+H'*W*H);
    
    est_x = xm;
    est_P = Pm;
    
    % book-keeping
    trueStateHistory(i+1,:) = x;
    kfEstHistory(i+1,:) = est_x;
    kfVarHistory(i+1,:) = diag(est_P);
    kfGainHistory(i+1,:) = K;
    measHistory(i+1,:) = z;
    inputHistory(i+1,:) = uf;
end

%% Steady state Kalman gain
K = kfGainHistory(end,:)';

est_x = [0;0];
sskfEstHistory = zeros(iter+1,2);     % state estimate
sskfEstHistory(1,:) = est_x;

for i = 1:iter
    uf = Force(FORCE_TYPE,i*dt,w,Amp); % desire force
    % prior update
    xp = Ad*est_x + Bd*uf;
    % measurement update
    xm = xp+K*(measHistory(i+1,:)-H*xp);
    est_x = xm;
    
    % book-keeping
    sskfEstHistory(i+1,:) = est_x;
end

%% ode45
% ----- state space representation -----
f = @(t,X)[X(2);1/m*Force(FORCE_TYPE,t,w,Amp)-k/m*X(1)-c/m*X(2)]; % sin input
% --------------------------------------

T = dt*iter; % sec
[ts,xs] = ode45(f,[0,T],x_init);

%% Plot
figure(1);
    
var_hull = 3;
% plot(dt*(0:iter),inputHistory);
% plot(ts,xs(:,1),'Color',[0.9290 0.6940 0.1250]); hold on
subplot(2,1,1);
h1 = plot(dt*(0:iter), trueStateHistory(:,1),'k','LineWidth',2); hold on
h2 = plot(dt*(0:iter), kfEstHistory(:,1),'b','LineWidth',2); hold on
h3 = plot(dt*(0:iter), kfEstHistory(:,1)+var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on
h4 = plot(dt*(0:iter), kfEstHistory(:,1)-var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on
h5 = plot(dt*(0:iter), measHistory,'r.');

% the following line skip the name of the previous plot from the legend
h4.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('True State','$\hat{x_m}$',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'measurement','Interpreter','latex');
xlabel('Time step k','Interpreter','latex');
ylabel('Position $x$','Interpreter','latex');

% title name
if NOISE_TYPE == 'nor'
    title('Noise is normal');
elseif NOISE_TYPE == 'uni'
    title('Noise is uniform');
elseif NOISE_TYPE == 'bin'
    title('Noise is binomial');
end

subplot(2,1,2);
plot(dt*(0:iter), trueStateHistory(:,2),'k','LineWidth',2); hold on
plot(dt*(0:iter), kfEstHistory(:,2),'b','LineWidth',2); hold on
plot(dt*(0:iter), kfEstHistory(:,2)+var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
plot(dt*(0:iter), kfEstHistory(:,2)-var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
xlabel('Time step k','Interpreter','latex');
ylabel('Velocity $\dot{x}$','Interpreter','latex');

figure(2)
subplot(2,1,1);
h1 = plot(dt*(0:iter), trueStateHistory(:,1)-kfEstHistory(:,1),'b','LineWidth',2); hold on
h2 = plot(dt*(0:iter), var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2);
h3 = plot(dt*(0:iter), -var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2);

% the following line skip the name of the previous plot from the legend
h3.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('estimation error',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'Interpreter','latex');
xlabel('Time step k','Interpreter','latex');
ylabel('Position error','Interpreter','latex');

% title name
if NOISE_TYPE == 'nor'
    title('Noise is normal');
elseif NOISE_TYPE == 'uni'
    title('Noise is uniform');
elseif NOISE_TYPE == 'bin'
    title('Noise is binomial');
end

subplot(2,1,2);
plot(dt*(0:iter), trueStateHistory(:,2)-kfEstHistory(:,2),'b','LineWidth',2); hold on
plot(dt*(0:iter), var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
plot(dt*(0:iter), -var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2);
xlabel('Time step k','Interpreter','latex');
ylabel('Velocity error','Interpreter','latex');

figure(3)
subplot(2,1,1);
plot(dt*(0:iter), kfGainHistory(:,1),'b','LineWidth',2);
xlabel('Time step k','Interpreter','latex');
ylabel('$Gain\ K_1$','Interpreter','latex');

subplot(2,1,2);
plot(dt*(0:iter), kfGainHistory(:,2),'b','LineWidth',2);
xlabel('Time step k','Interpreter','latex');
ylabel('$Gain\ K_2$','Interpreter','latex');

figure(4)
subplot(2,1,1);
plot(dt*(0:iter), trueStateHistory(:,1),'k','LineWidth',2); hold on
plot(dt*(0:iter), kfEstHistory(:,1),'b','LineWidth',2); hold on
plot(dt*(0:iter), sskfEstHistory(:,1),'Color',[0.9290 0.6940 0.1250],'LineWidth',2);
plot(dt*(0:iter), kfEstHistory(:,1)+var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on
h = plot(dt*(0:iter), kfEstHistory(:,1)-var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on
% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
xlabel('Time step k','Interpreter','latex');
ylabel('Position $x$','Interpreter','latex');
legend('True State','time varying Kalman Filter','steady state Kalman Filter',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'Interpreter','latex');

% title name
if NOISE_TYPE == 'nor'
    title('Noise is normal');
elseif NOISE_TYPE == 'uni'
    title('Noise is uniform');
elseif NOISE_TYPE == 'bin'
    title('Noise is binomial');
end

subplot(2,1,2);
plot(dt*(0:iter), trueStateHistory(:,2),'k','LineWidth',2); hold on
plot(dt*(0:iter), kfEstHistory(:,2),'b','LineWidth',2); hold on
plot(dt*(0:iter), sskfEstHistory(:,2),'Color',[0.9290 0.6940 0.1250],'LineWidth',2);
plot(dt*(0:iter), kfEstHistory(:,2)+var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
h = plot(dt*(0:iter), kfEstHistory(:,2)-var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
xlabel('Time step k','Interpreter','latex');
ylabel('Velocity $\dot{x}$','Interpreter','latex');

figure(5)
subplot(2,1,1);
plot(dt*(0:iter), trueStateHistory(:,1)-kfEstHistory(:,1),'b','LineWidth',2); hold on
plot(dt*(0:iter), trueStateHistory(:,1)-sskfEstHistory(:,1),'Color',[0.9290 0.6940 0.1250],'LineWidth',2); hold on
plot(dt*(0:iter), var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on
h = plot(dt*(0:iter), -var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2);
% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('KF estimation error','ss KF estimation error',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'Interpreter','latex');
xlabel('Time step k','Interpreter','latex');
ylabel('Position error','Interpreter','latex');

% title name
if NOISE_TYPE == 'nor'
    title('Noise is normal');
elseif NOISE_TYPE == 'uni'
    title('Noise is uniform');
elseif NOISE_TYPE == 'bin'
    title('Noise is binomial');
end

subplot(2,1,2);
h1 = plot(dt*(0:iter), trueStateHistory(:,2)-kfEstHistory(:,2),'b','LineWidth',2); hold on
h2 = plot(dt*(0:iter), trueStateHistory(:,2)-sskfEstHistory(:,2),'Color',[0.9290 0.6940 0.1250],'LineWidth',2); hold on
h3 = plot(dt*(0:iter), var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
h4 = plot(dt*(0:iter), -var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2);
% the following line skip the name of the previous plot from the legend
h4.Annotation.LegendInformation.IconDisplayStyle = 'off';
xlabel('Time step k','Interpreter','latex');
ylabel('Velocity error','Interpreter','latex');

%% functions
function noise = noiseModel(type, mean, variance)
    if type == 'uni'
        noise = unifrnd(-3*sqrt(variance),3*sqrt(variance))+mean;
    end
    if type == 'nor'
        noise = normrnd(mean, variance);
    end
    if type == 'bin'
        if unifrnd(0,1) < 0.5
            noise = mean + sqrt(variance);
        else
            noise = mean - sqrt(variance);
        end
    end
end


function f = Force(type,t,w,A)
    if type == 'sine'
        f = A*sin(w*t);
    elseif type == 'single_sine'
        if t <= 2*pi/w
            f = A*sin(w*t);
        else
            f = 0;
        end
    end
end