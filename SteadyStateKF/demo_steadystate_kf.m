clear all; close all; clc;

% CASE = 1;
% CASE = 2;
% CASE = 3;
% CASE = 4;
CASE = 5;
% CASE = 6;
% CASE = 7;

%% Parameters
% assume process noise of states are uncorrelated(diagonal matrix)
r = 10; % measurement noise variance remain the same for all cases
if CASE == 1
    A = [2 1 ; 0 0.5];
    H = [1 0];
    q = 1; % process noise
    initStateVar = eye(2); % prior knowledge of state
end
if CASE == 2
    A = [2 1 ; 0 0.5];
    H = [0 1];
    q = 1;
    initStateVar = eye(2);
end
if CASE == 3
    A = [0.5 1 ; 0 2];
    H = [0 1];
    q = 1;
    initStateVar = eye(2);
end
if CASE == 4
    A = [0 1; -2 0];
    H = [0 1];
    q = 0; % no process noise
    initStateVar = [0 0 ; 0 1];
end
if CASE == 5
    A = [0 1 ; -2 0];
    H = [0 1];
    q = 0;
    initStateVar = eye(2);
end
if CASE == 6
    A = [0 1;-2 0];
    H = [0 1];
    q = 1e-12;
    initStateVar = [0 0 ; 0 1];
end
if CASE == 7
    A = [0 1;-2 0];
    H = [0 1];
    q = 0;
    initStateVar = [1 0 ; 0 0];
end

%% Kalman Filter
iter = 60;

% ----- estimator initialization -----
est_x = [0;0];
est_P = initStateVar;
% ------------------------------------

% ----- True State -----
true_x = [noiseModel(0,initStateVar(1,1));noiseModel(0,initStateVar(2,2))];
% ----------------------

% ----- book-keeping-----
trueStateHistory = zeros(iter+1,2);
kfEstHistory = zeros(iter+1,2);
kfVarHistory = zeros(iter+1,2);
kfGainHistory = zeros(iter+1,2);
kf_Pp_History = zeros(iter+1,4);
measHistory = zeros(iter+1,1);

% initial value
trueStateHistory(1,:) = true_x;
kfEstHistory(1,:) = est_x;
kfVarHistory(1,:) = diag(est_P);
kfGainHistory(1,:) = nan;
kf_Pp_History(1,:) = nan;
measHistory(1,:) = nan;

for i = 1:iter
    % True System Dynamic
    true_x = A*true_x + [noiseModel(0,q);noiseModel(0,q)];
    
    % measurement
    z = H*true_x + noiseModel(0,1);
    
    % ----- Kalman Filter -----
    % prior update
    xp = A*est_x;
    Pp = A*est_P*A'+q*eye(2);
    
    % measurement update
    K = Pp*H'*inv(H*Pp*H'+r);
    xm = xp + K*(z-H*xp);
    Pm = (eye(2)-K*H)*Pp;
    
    est_x = xm;
    est_P = Pm;
    
    % ----- book-keeping -----
    trueStateHistory(i+1,:) = true_x;
    kfEstHistory(i+1,:) = est_x;
    kfVarHistory(i+1,:) = diag(est_P);
    kfGainHistory(i+1,:) = K;
    kf_Pp_History(i+1,:) = reshape(Pp.',1,[]);
    measHistory(i+1,:) = z;
end

%% plot
var_hull = 1;

figure(1);
subplot(2,1,1);
plot((0:iter), trueStateHistory(:,1),'k','LineWidth',1); hold on
plot((0:iter), kfEstHistory(:,1),'b','LineWidth',1); hold on
plot((0:iter), kfEstHistory(:,1)+var_hull*sqrt(kfVarHistory(:,1)),'g--','LineWidth',1); hold on
h = plot((0:iter), kfEstHistory(:,1)-var_hull*sqrt(kfVarHistory(:,1)),'g--','LineWidth',1); hold on
% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
if(H(1))
    plot((0:iter), measHistory,'r.');
    legend('True State','$\hat{x_{m,1}}$',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'measurement','Interpreter','latex');
end
xlabel('Time step k','Interpreter','latex');
ylabel('$\hat{x_{m,1}}$','Interpreter','latex');

subplot(2,1,2);
plot((0:iter), trueStateHistory(:,2),'k','LineWidth',2); hold on
plot((0:iter), kfEstHistory(:,2),'b','LineWidth',2); hold on
plot((0:iter), kfEstHistory(:,2)+var_hull*sqrt(kfVarHistory(:,2)),'g--','LineWidth',2); hold on
h = plot((0:iter), kfEstHistory(:,2)-var_hull*sqrt(kfVarHistory(:,2)),'g--','LineWidth',2); hold on
% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
if(H(2))
    plot((0:iter), measHistory,'r.');
    legend('True State','$\hat{x_{m,1}}$',['Covariance Hull(',num2str(var_hull),'$\sigma$)'],'measurement','Interpreter','latex');
end
xlabel('Time step k','Interpreter','latex');
ylabel('$\hat{x_{m,2}}$','Interpreter','latex');

% ----- plot error and variance hull -----
figure(2);
subplot(2,1,1);
plot((0:iter), trueStateHistory(:,1)-kfEstHistory(:,1),'b','LineWidth',2); hold on
plot((0:iter), var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on
h = plot((0:iter), -var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on

% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('error',['Variance Hull(',num2str(var_hull),'$\sigma$)'],'Interpreter','latex');

xlabel('Time Step k','Interpreter','latex');
ylabel('$x_1$ error','Interpreter','latex');

subplot(2,1,2);
plot((0:iter),trueStateHistory(:,2)-kfEstHistory(:,2),'b','LineWidth',2); hold on
plot((0:iter),var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
plot((0:iter),-var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
xlabel('Time Step k','Interpreter','latex');
ylabel('$x_2$ error','Interpreter','latex');

% ----- plot Pp History -----
figure(3);
subplot(2,1,1);
plot((0:iter), kf_Pp_History(:,1),'r','LineWidth',2); hold on
plot((0:iter), kf_Pp_History(:,2),'b','LineWidth',2); hold on
plot((0:iter), kf_Pp_History(:,4),'g','LineWidth',2); hold on
xlabel('Time Step k','Interpreter','latex');
ylabel('$P_p$','Interpreter','latex');
legend('$P_p^{(1,1)}$','$P_p^{(1,2)}$','$P_p^{(2,2)}$','Interpreter','latex');

subplot(2,1,2);
plot((0:iter), kfGainHistory(:,1),'Color',[0.8500 0.3250 0.0980],'LineWidth',2); hold on
plot((0:iter), kfGainHistory(:,2),'Color',[0 0.4470 0.7410],'LineWidth',2); hold on
xlabel('Time Step k','Interpreter','latex');
ylabel('Kalman Gain','Interpreter','latex');
legend('$K_1$','$K_2$','Interpreter','latex');

figure(4);
subplot(3,1,1);
plot((0:iter), trueStateHistory(:,1)-kfEstHistory(:,1),'b','LineWidth',2); hold on
plot((0:iter), var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on
h = plot((0:iter), -var_hull*sqrt(kfVarHistory(:,1)),'g','LineWidth',2); hold on

% the following line skip the name of the previous plot from the legend
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('error',['Variance Hull(',num2str(var_hull),'$\sigma$)'],'Interpreter','latex');

xlabel('Time Step k','Interpreter','latex');
ylabel('$x_1$ error','Interpreter','latex');

subplot(3,1,2);
plot((0:iter),trueStateHistory(:,2)-kfEstHistory(:,2),'b','LineWidth',2); hold on
plot((0:iter),var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
plot((0:iter),-var_hull*sqrt(kfVarHistory(:,2)),'g','LineWidth',2); hold on
xlabel('Time Step k','Interpreter','latex');
ylabel('$x_2$ error','Interpreter','latex');

subplot(3,1,3);
plot((0:iter), kf_Pp_History(:,1),'Color',[0.8500 0.3250 0.0980],'LineWidth',2); hold on
plot((0:iter), kf_Pp_History(:,2),'Color',[0.4660 0.6740 0.1880],'LineWidth',2); hold on
plot((0:iter), kf_Pp_History(:,4),'Color',[0 0.4470 0.7410],'LineWidth',2); hold on
xlabel('Time Step k','Interpreter','latex');
ylabel('$P_p$','Interpreter','latex');
legend('$P_p^{(1,1)}$','$P_p^{(1,2)}$','$P_p^{(2,2)}$','Interpreter','latex');


%% function
function noise = noiseModel(mean, variance)
    noise = normrnd(mean, variance); 
end




