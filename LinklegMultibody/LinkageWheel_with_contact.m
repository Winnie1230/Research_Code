%% Solve Multibody System with LCP methods(Assume to be frictionless)
clear all; close all; clc;

% Linkage Order
% (1)Body
% (2)alpha1
% (3)alpha2
% (4)gamma1
% (5)gamma2
% (6)beta1
% (7)beta2
% (8)delta1
% (9)delta2
% (10)epsilon1
% (11)epsilon2

%% General Parameters (Using metic units - mmgs)

% Time
time.period = 0.002;
time.step = 50;

% Useful Constants
n = 11; % Number of linkages

%Environment
g = 9810 ;      % Gravitaional acceleration

% Constraints
no = 5; % Number of constraints to constrain Body to move vertically only
nc = 14; % Number of joints(hinge)
n_fix = 0; % Number of constraints to restrict the rotation around y-axis between alpha1 and O, alpha2 and O
% ball-and-socket joint only need a position constraint for translation
% 3 constraints for epsilon1 and 2 repectively
ng_fix = 0; % fix pointG to the ground(= regard pointG as Hinge Joint or Ball-And-Socket Joint?)

% plot
aa = 0.7;
bb = 0.7;
cc = 3;

% plot animation(true/false)
PLOT_ANIMATION = true;
% PLOT_ANIMATION = false;

% load R100 parameter
load('param_R100.mat','R','up_deg');

% gif
del = 0.01; % time between animation frames

% create the video writer with 1 fps
% writerObj = VideoWriter('myVideo_fixed.avi');
% writerObj.FrameRate = 100;
% set the seconds per image
% open the video writer
% open(writerObj);

%% torque command
Amp = 5;  % Nm
wt = 5; % Hz sin frequency

duty = 1;
torque = 2.5;
km = 2.41338; % N.m/A current to torque

%% Initial Conditions
% Computing points and angle between linkage and [1 0] according to initial theta
beta_deg = 0;
theta_deg = 30;
points = calculate_coordinate_flat2(theta_deg,beta_deg);
angle = linkleg_angle(points); % compute angle between +x dir and use it to setup the initial orientation of every linkage

% set z-dir of point G to 0; (shift all coordinate)
% Export the output coordinates in 3D
shiftG = [0;0;points.G_3d(3)];
points.O_3d  = points.O_3d  - shiftG;
points.A1_3d = points.A1_3d - shiftG;
points.A2_3d = points.A2_3d - shiftG;
points.B1_3d = points.B1_3d - shiftG;
points.B2_3d = points.B2_3d - shiftG;
points.C1_3d = points.C1_3d - shiftG;
points.C2_3d = points.C2_3d - shiftG;
points.D1_3d = points.D1_3d - shiftG;
points.D2_3d = points.D2_3d - shiftG;
points.E_3d = points.E_3d   - shiftG;
points.F1_3d = points.F1_3d - shiftG;
points.F2_3d = points.F2_3d - shiftG;
points.G_3d = points.G_3d   - shiftG;

%% Linkage configurations (Using metic units - mmgs)
load('LinkProperty_R100.mat');

% ---------- Mass BASE(B) ----------
BASE.x = points.O_3d;
BASE.theta = 0;
BASE.q = [cos(BASE.theta/2) 0 0 sin(BASE.theta/2)]'; % Orientation of mass B (quaternion)

% sliding joint
BASE.anchor = [0 0 -100]';
% B.x_anchor = [0 0 -100]; % CoM to anchor point


% ---------- Linkage alpha-1(Rigid Body) ----------
alpha1.q = [cos(angle.alpha1_phi/2) 0 -sin(angle.alpha1_phi/2) 0]';
% alpha1.alpha1_O1 = [-36.45 -8.63 0.02]'; % local frame
alpha1.x = Rq(BASE.q)*BASE.x - Rq(alpha1.q)*alpha1.alpha1_O1; % global frame
alpha1.alpha1_A1 = Rq(alpha1.q)'*(Rq(BASE.q)*points.A1_3d - alpha1.x); % local frame
alpha1.alpha1_B1 = Rq(alpha1.q)'*(Rq(BASE.q)*points.B1_3d - alpha1.x); % local frame
% --------------------------------------------------

% ---------- Linkage alpha-2(Rigid Body) -----------
alpha2.q = [cos(angle.alpha2_phi/2) 0 -sin(angle.alpha2_phi/2) 0]';
alpha2.x = Rq(BASE.q)*BASE.x - Rq(alpha2.q)*alpha2.alpha2_O2; % global frame
alpha2.alpha2_A2 = Rq(alpha2.q)'*(Rq(BASE.q)*points.A2_3d - alpha2.x); % local frame
alpha2.alpha2_B2 = Rq(alpha2.q)'*(Rq(BASE.q)*points.B2_3d - alpha2.x); % local frame
% ----------------------------------------------------

% ---------- Linkage gamma-1(Rigid Body) ----------
gamma1.q = [cos(angle.gamma1_phi/2) 0 -sin(angle.gamma1_phi/2) 0]';
gamma1.x = Rq(BASE.q)*points.A1_3d - Rq(gamma1.q)*gamma1.gamma1_A1; % global frame
gamma1.gamma1_D1 = Rq(gamma1.q)'*(Rq(BASE.q)*points.D1_3d - gamma1.x); % local frame
gamma1.gamma1_E  = Rq(gamma1.q)'*(Rq(BASE.q)*points.E_3d - gamma1.x); % local frame
% -------------------------------------------------


% ---------- Linkage gamma-2(Rigid Body) ----------
gamma2.q = [cos(angle.gamma2_phi/2) 0 -sin(angle.gamma2_phi/2) 0]';
gamma2.x = Rq(BASE.q)*points.A2_3d - Rq(gamma2.q)*gamma2.gamma2_A2;  % global frame
gamma2.gamma2_D2 = Rq(gamma2.q)'*(Rq(BASE.q)*points.D2_3d - gamma2.x); % local frame
gamma2.gamma2_E  = Rq(gamma2.q)'*(Rq(BASE.q)*points.E_3d - gamma2.x); % local frame
% -------------------------------------------------

% ----------- Linkage beta-1(Rigid Body) ----------
beta1.q = [cos(angle.beta1_phi/2) 0 -sin(angle.beta1_phi/2) 0]';
beta1.x = Rq(BASE.q)*points.B1_3d - Rq(beta1.q)*beta1.beta1_B1; % global frame
beta1.beta1_C1 = Rq(beta1.q)'*(Rq(BASE.q)*points.C1_3d - beta1.x); % local frame
beta1.beta1_F1 = Rq(beta1.q)'*(Rq(BASE.q)*points.F1_3d - beta1.x); % local frame
% -------------------------------------------------

% ----------- Linkage beta-2(Rigid Body) ----------
beta2.q = [cos(angle.beta2_phi/2) 0 -sin(angle.beta2_phi/2) 0]';
beta2.x = Rq(BASE.q)*points.B2_3d - Rq(beta2.q)*beta2.beta2_B2; % global frame
beta2.beta2_C2 = Rq(beta2.q)'*(Rq(BASE.q)*points.C2_3d - beta2.x); % local frame
beta2.beta2_F2 = Rq(beta2.q)'*(Rq(BASE.q)*points.F2_3d - beta2.x); % local frame
% -------------------------------------------------

% ---------- Linkage delta-1(Rigid Body) ----------
delta1.q = [cos(angle.delta1_phi/2) 0 -sin(angle.delta1_phi/2) 0]';
delta1.x = Rq(BASE.q)*points.D1_3d - Rq(delta1.q)*delta1.delta1_D1; % global frame
delta1.delta1_C1 = Rq(delta1.q)'*(Rq(BASE.q)*points.C1_3d - delta1.x); % local frame
% -------------------------------------------------

% ---------- Linkage delta-2(Rigid Body) ----------
delta2.q = [cos(angle.delta2_phi/2) 0 -sin(angle.delta2_phi/2) 0]'; 
delta2.x = Rq(BASE.q)*points.D2_3d - Rq(delta2.q)*delta2.delta2_D2; % global frame
delta2.delta2_C2 = Rq(delta2.q)'*(Rq(BASE.q)*points.C2_3d - delta2.x); % local frame
% -------------------------------------------------

% ---------- Linkage epsilon-1(Rigid Body) ----------
epsilon1.q = [cos(angle.epsilon1_phi/2) 0 -sin(angle.epsilon1_phi/2) 0]';
epsilon1.x = Rq(BASE.q)*points.F1_3d - Rq(epsilon1.q)*epsilon1.epsilon1_F1; % global frame
epsilon1.epsilon1_G = Rq(epsilon1.q)'*(Rq(BASE.q)*points.G_3d - epsilon1.x); % local frame
% ---------------------------------------------------

% ---------- Linkage epsilon-2(Rigid Body) ----------
epsilon2.q = [cos(angle.epsilon2_phi/2) 0 -sin(angle.epsilon2_phi/2) 0]';
epsilon2.x = Rq(BASE.q)*points.F2_3d - Rq(epsilon2.q)*epsilon2.epsilon2_F2; % global frame
epsilon2.epsilon2_G = Rq(epsilon2.q)'*(Rq(BASE.q)*points.G_3d - epsilon2.x); % local frame
% ---------------------------------------------------

% hold on
% plot3(points.O_3d(1),points.O_3d(2),points.O_3d(3),'ko','MarkerFaceColor','k','MarkerEdgeColor','none');
% plot3(points.A1_3d(1),points.A1_3d(2),points.A1_3d(3),'bo','MarkerFaceColor','b','MarkerEdgeColor','none');
% plot3(points.A2_3d(1),points.A2_3d(2),points.A2_3d(3),'go','MarkerFaceColor','g','MarkerEdgeColor','none');
% plot3(points.B1_3d(1),points.B1_3d(2),points.B1_3d(3),'bo','MarkerFaceColor','b','MarkerEdgeColor','none');
% plot3(points.B2_3d(1),points.B2_3d(2),points.B2_3d(3),'go','MarkerFaceColor','g','MarkerEdgeColor','none');
% plot3(points.C1_3d(1),points.C1_3d(2),points.C1_3d(3),'bo','MarkerFaceColor','b','MarkerEdgeColor','none');
% plot3(points.C2_3d(1),points.C2_3d(3),points.C2_3d(3),'go','MarkerFaceColor','g','MarkerEdgeColor','none');
% plot3(points.D1_3d(1),points.D1_3d(2),points.D1_3d(3),'bo','MarkerFaceColor','b','MarkerEdgeColor','none');
% plot3(points.D2_3d(1),points.D2_3d(2),points.D2_3d(3),'go','MarkerFaceColor','g','MarkerEdgeColor','none');
% plot3(points.F1_3d(1),points.F1_3d(2),points.F1_3d(3),'bo','MarkerFaceColor','b','MarkerEdgeColor','none');
% plot3(points.F2_3d(1),points.F2_3d(2),points.F2_3d(3),'go','MarkerFaceColor','g','MarkerEdgeColor','none');
% plot3(points.E_3d(1),points.E_3d(2),points.E_3d(3),'ro','MarkerFaceColor','r','MarkerEdgeColor','none');
% plot3(points.G_3d(1),points.G_3d(2),points.G_3d(3),'ro','MarkerFaceColor','r','MarkerEdgeColor','none');
% 
% plot3(alpha1.x(1),alpha1.x(2),alpha1.x(3),'mo','MarkerFaceColor','m','MarkerEdgeColor','none');
% plot3(alpha2.x(1),alpha2.x(2),alpha2.x(3),'co','MarkerFaceColor','c','MarkerEdgeColor','none');
% plot3(gamma1.x(1),gamma1.x(2),gamma1.x(3),'mo','MarkerFaceColor','m','MarkerEdgeColor','none');
% plot3(gamma2.x(1),gamma2.x(2),gamma2.x(3),'co','MarkerFaceColor','c','MarkerEdgeColor','none');
% plot3(beta1.x(1),beta1.x(2),beta1.x(3),'mo','MarkerFaceColor','m','MarkerEdgeColor','none');
% plot3(beta2.x(1),beta2.x(2),beta2.x(3),'co','MarkerFaceColor','c','MarkerEdgeColor','none');
% plot3(delta1.x(1),delta1.x(2),delta1.x(3),'mo','MarkerFaceColor','m','MarkerEdgeColor','none');
% plot3(delta2.x(1),delta2.x(2),delta2.x(3),'co','MarkerFaceColor','c','MarkerEdgeColor','none');
% plot3(epsilon1.x(1),epsilon1.x(2),epsilon1.x(3),'mo','MarkerFaceColor','m','MarkerEdgeColor','none');
% plot3(epsilon2.x(1),epsilon2.x(2),epsilon2.x(3),'co','MarkerFaceColor','c','MarkerEdgeColor','none');
% 
% view(0,0);
% axis equal
% axis([-200 200 -200 200 -100 350]);
% hold off


% Combining the unit vectors for convenience
% ns = [BASE.nx ... alpha1.nx ... alpha2.nx ... beta1.nx ........epsilon2.nz]
ns_ = zeros(3,3,n);
for i = 1 : n
    ns_(:,:,i) = eye(3);  % [ nx ny nz ] = eye(3)
end

% M = [B.m];
% I = [B.I];
% M = [B.m alpha1.m alpha2.m];
% I = [B.I alpha1.I alpha2.I];
% M = [B.m alpha1.m alpha2.m gamma1.m gamma2.m];
% I = [B.I alpha1.I alpha2.I gamma1.I gamma2.I];
% M = [B.m alpha1.m alpha2.m gamma1.m gamma2.m beta1.m beta2.m delta1.m delta2.m];
% I = [B.I alpha1.I alpha2.I gamma1.I gamma2.I beta1.I beta2.I delta1.I delta2.I];
M = [BASE.m alpha1.m alpha2.m gamma1.m gamma2.m beta1.m beta2.m delta1.m delta2.m epsilon1.m epsilon2.m];
I = [BASE.I alpha1.I alpha2.I gamma1.I gamma2.I beta1.I beta2.I delta1.I delta2.I epsilon1.I epsilon2.I];

%% Book-keeping
history.s = zeros(7*n,time.step+1);
history.v = zeros(6*n,time.step+1);
history.J = zeros(5*nc + no + n_fix + ng_fix,6*n,time.step+1);
history.lambda = zeros(5*nc + no + n_fix + ng_fix,time.step+1);
history.torque = zeros(time.step+1,1);
history.Fn = zeros(time.step+1,1);
history.theta = zeros(time.step+1,1);
history.O = zeros(time.step+1,3);
history.Pinertial = zeros(time.step+1,1);
history.Pg = zeros(time.step+1,1); % work done by gravitational force
history.PFext = zeros(time.step+1,1);
history.PFn = zeros(time.step+1,1);
history.Pinertial_BASE = zeros(time.step+1,1);
history.Rmasscenter = zeros(time.step+1,3);
history.Lmasscenter = zeros(time.step+1,3);
history.total_masscenter = zeros(time.step+1,3);

%% Initialization
% system states: (linkage C.O.M position & quaternion in world frame)
% s = (BASE.x,BASE.q,alpha1.x,alpha1.q,...)
% velocity: (C.O.M velocities in world frame)
% v = (BASE.v,BASE.w,alpha1.v,alpha1.w,...)

% mass matrix
MassMatrix = zeros(6*n, 6*n);

% inverse mass matrix
W = zeros(6*n,6*n);

% externel wrench input
wrench.ext = zeros(6*n,1);
% wrench.ext(11,1) = 0.767*2.41338*1e+9;
% wrench.ext(17,1) = -0.767*2.41338*1e+9;

% non torque term of the euler's of motion
wrench.w = zeros(6*n,1);

% Gravitational force
wrench.g = zeros(6,n);
wrench.g(3,:) = M*(-g);
wrench.g = reshape(wrench.g,6*n,1);

% Reactional force factor (Largrange Multiplier)
lambda = zeros(5*nc + no + n_fix + ng_fix,1);
% lambda = zeros(5*nc + no,1);
wrench.react = zeros(6*n,1);
wrench.total = zeros(6*n,1);

%% Initial Conditions and Setup / Initialize book-keeping
s0 = [    BASE.x ;        BASE.q ;
        alpha1.x ;   alpha1.q ;   alpha2.x ;   alpha2.q ;
        gamma1.x ;   gamma1.q ;   gamma2.x ;   gamma2.q ;  
         beta1.x ;    beta1.q ;    beta2.x ;    beta2.q ; 
        delta1.x ;   delta1.q ;   delta2.x ;   delta2.q ; 
      epsilon1.x ; epsilon1.q ; epsilon2.x ; epsilon2.q];
% s0 = [B.x ; B.q ; alpha1.x ; alpha1.q ; alpha2.x ; alpha2.q ; gamma1.x ; gamma1.q ; gamma2.x ; gamma2.q ; beta1.x ; beta1.q ; beta2.x ; beta2.q ; delta1.x ; delta1.q ; delta2.x ; delta2.q];
% s0 = [B.x ; B.q ; alpha1.x ; alpha1.q ; alpha2.x ; alpha2.q ; gamma1.x ; gamma1.q ; gamma2.x ; gamma2.q];
% s0 = [B.x ; B.q ; alpha1.x ; alpha1.q ; alpha2.x ; alpha2.q];
% s0 = [B.x ; B.q];
v0 = zeros(6*n,1);
s = s0;
v = v0;

% ---------- initial conditions ----------
% vector: vector pointing from linkage CoM to hinge joint 

% Linkage alpha1
vector.alpha1_O1 = Rq(s(11:14))*alpha1.alpha1_O1;
vector.alpha1_A1 = Rq(s(11:14))*alpha1.alpha1_A1;
vector.alpha1_B1 = Rq(s(11:14))*alpha1.alpha1_B1;

% Linkage alpha2
vector.alpha2_O2 = Rq(s(18:21))*alpha2.alpha2_O2;
vector.alpha2_A2 = Rq(s(18:21))*alpha2.alpha2_A2;
vector.alpha2_B2 = Rq(s(18:21))*alpha2.alpha2_B2;

% Linkage gamma1
vector.gamma1_A1 = Rq(s(25:28))*gamma1.gamma1_A1;
vector.gamma1_D1 = Rq(s(25:28))*gamma1.gamma1_D1;
vector.gamma1_E  = Rq(s(25:28))*gamma1.gamma1_E;

% Linkage gamma2
vector.gamma2_A2 = Rq(s(32:35))*gamma2.gamma2_A2;
vector.gamma2_D2 = Rq(s(32:35))*gamma2.gamma2_D2;
vector.gamma2_E  = Rq(s(32:35))*gamma2.gamma2_E;

% Linkage beta1
vector.beta1_B1 = Rq(s(39:42))*beta1.beta1_B1;
vector.beta1_C1 = Rq(s(39:42))*beta1.beta1_C1;
vector.beta1_F1 = Rq(s(39:42))*beta1.beta1_F1;

% Linkage beta2
vector.beta2_B2 = Rq(s(46:49))*beta2.beta2_B2;
vector.beta2_C2 = Rq(s(46:49))*beta2.beta2_C2;
vector.beta2_F2 = Rq(s(46:49))*beta2.beta2_F2;

% Linkage delta1
vector.delta1_C1 = Rq(s(53:56))*delta1.delta1_C1;
vector.delta1_D1 = Rq(s(53:56))*delta1.delta1_D1;

% Linkage delta2
vector.delta2_C2 = Rq(s(60:63))*delta2.delta2_C2;
vector.delta2_D2 = Rq(s(60:63))*delta2.delta2_D2;

% Linkage epsilon1
vector.epsilon1_F1 = Rq(s(67:70))*epsilon1.epsilon1_F1;
vector.epsilon1_G  = Rq(s(67:70))*epsilon1.epsilon1_G;

% Linkage epsilon2
vector.epsilon2_F2 = Rq(s(74:77))*epsilon2.epsilon2_F2;
vector.epsilon2_G  = Rq(s(74:77))*epsilon2.epsilon2_G;
% ----------------------------------------

% initialize book-keeping
history.s(:,1) = s;
history.v(:,1) = v;
history.theta(1,:) = theta_deg;
history.O(1,:) = BASE.x;
history.Rmasscenter(1,:) = (alpha1.m*alpha1.x + beta1.m*beta1.x + gamma1.m*gamma1.x + delta1.m*delta1.x + epsilon1.m*epsilon1.x)/(alpha1.m + beta1.m + gamma1.m + delta1.m + epsilon1.m);
history.Lmasscenter(1,:) = (alpha2.m*alpha2.x + beta2.m*beta2.x + gamma2.m*gamma2.x + delta2.m*delta2.x + epsilon2.m*epsilon2.x)/(alpha2.m + beta2.m + gamma2.m + delta2.m + epsilon2.m);
history.total_masscenter(1,:) = (BASE.m*BASE.x + alpha1.m*alpha1.x + beta1.m*beta1.x + gamma1.m*gamma1.x + delta1.m*delta1.x + epsilon1.m*epsilon1.x + alpha2.m*alpha2.x + beta2.m*beta2.x + gamma2.m*gamma2.x + delta2.m*delta2.x + epsilon2.m*epsilon2.x)/(sum(M));

% animation
% figure
% subplot(2,2,2);
% h1 = animatedline('Color','b','LineWidth',1);
% h2 = animatedline('Color','r','LineWidth',1);
% xlim([0 time.period*(index.Vicon_LO-index.Motor_startVJ+1)]);
% % axis([0 time.period*time.step -Amp Amp]);
% title('torque-t');
% xlabel('$t[sec]$','Interpreter','latex');
% ylabel('$torque[N\cdot m]$','Interpreter','latex');
% 
% subplot(2,2,4);
% h3 = animatedline('Color','b','LineWidth',1);
% h4 = animatedline('Color','r','LineWidth',1);
% xlim([0 time.period*(index.Vicon_LO-index.Motor_startVJ+1)]);
% % axis([0 time.period*time.step -Amp Amp]);
% title('Fn-t');
% xlabel('$t[sec]$','Interpreter','latex');
% ylabel('$Fn[N]$','Interpreter','latex');

if(PLOT_ANIMATION)
    gcf_arr = [0.3,0.2,0.50,0.70];
    % set(gcf,'unit','normalized','position',gcf_arr);
    % set(gcf,'papersize',[15.2,8.4])
    set(gcf,'paperposition',[-2,0.05,15.2,8.4]);
end

for t = 1:time.step
    tic
    %% Rotate inertia tensor
    I_rot = zeros(3,3*n);
    for i = 0 : n-1
        I_rot(:, 3*i + 1 : 3*i + 3) = Rq(s(7*i+4:7*(i+1),1)) * I(:,3*i+1:3*(i+1)) * Rq(s(7*i+4:7*(i+1),1))';
        I_rot(:, 3*i + 1 : 3*i + 3) = I_rot(:, 3*i + 1 : 3*i + 3  ) - tril(I_rot(:, 3*i + 1 : 3*i + 3  ),-1) + triu(I_rot(:, 3*i + 1 : 3*i + 3  ),1)';
    end

    %% Construct the mass matrix
    for i = 0 : n-1
        MassMatrix(i*6+1 : i*6+6, i*6+1 : i*6+6) = [M(i+1)*eye(3) zeros(3,3) ; zeros(3,3)  I(:,i*3+1 : i*3+3)];
    end

    %% Construct the inverse mass matrix
    for i = 1 : n
        W((i-1)*6+1 : i*6, (i-1)*6+1 : i*6) = [1/M(i)*eye(3) zeros(3,3) ; zeros(3,3) inv(I_rot(:,(i-1)*3+1 : i*3))];
    end

    %% Construct the matrix S
    % time derivative of a position vector and quaternion
    S = zeros(7*n,6*n);
    for i = 0 : n-1
        S(i*7+1 : i*7+3, i*6+1 : i*6+3) = eye(3);
        S(i*7+4 : i*7+7, i*6+4 : i*6+6) = Qq(s(i*7+3 : i*7+7,1));
    end

    %% rotate ns_
    ns = zeros(3,3,n);
    for i = 0 : n-1
        ns(:,:,i+1) = Rq(s(7*i+4 : 7*i+7,1))*ns_(:,:,i+1);
    end

    %% Refresh wrenches
    for i = 0 : n-1
        w = v(6*i+4:6*(i+1),1);
        wrench.w(6*i+4:6*(i+1),1) = -skew(w) * I_rot(:,3*i+1 : 3*(i+1)) * w;
    end

%     temp = ns(:,2,1);
%     temp2 = -vector.alpha1_O1/norm(vector.alpha1_O1);
%     wrench.ext(11,1) = torque*1e+9;
%     wrench.ext(17,1) = -torque*1e+9;

    % rotate about O1(inertia tensor of a1 + parallel axis theorem)
    I_a1_O1 = I_rot(:,4:6) + alpha1.m*[ (vector.alpha1_O1(2)^2+vector.alpha1_O1(3)^2)  -(vector.alpha1_O1(1)*vector.alpha1_O1(2))     -(vector.alpha1_O1(1)*vector.alpha1_O1(3));
                                       -(vector.alpha1_O1(1)*vector.alpha1_O1(2))       (vector.alpha1_O1(1)^2+vector.alpha1_O1(3)^2) -(vector.alpha1_O1(2)*vector.alpha1_O1(3));
                                       -(vector.alpha1_O1(1)*vector.alpha1_O1(3))      -(vector.alpha1_O1(2)*vector.alpha1_O1(3))      (vector.alpha1_O1(1)^2+vector.alpha1_O1(2)^2)];

    Torq_a1 = I_rot(:,4:6)*(I_a1_O1\(torque*1e+9*ns(:,2,1)-skew(v(10:12,1))*I_a1_O1*v(10:12,1))) + skew(v(10:12,1))*I_rot(:,4:6)*v(10:12,1);

    % rotate about O1(inertia tensor of a2 + parallel axis theorem)
    I_a2_O2 = I_rot(:,7:9) + alpha2.m*[ (vector.alpha2_O2(2)^2+vector.alpha2_O2(3)^2)  -(vector.alpha2_O2(1)*vector.alpha2_O2(2))     -(vector.alpha2_O2(1)*vector.alpha2_O2(3));
                                       -(vector.alpha2_O2(1)*vector.alpha2_O2(2))       (vector.alpha2_O2(1)^2+vector.alpha2_O2(3)^2) -(vector.alpha2_O2(2)*vector.alpha2_O2(3));
                                       -(vector.alpha2_O2(1)*vector.alpha2_O2(3))      -(vector.alpha2_O2(2)*vector.alpha2_O2(3))      (vector.alpha2_O2(1)^2+vector.alpha2_O2(2)^2)];

    Torq_a2 = I_rot(:,7:9)*(I_a2_O2\(-torque*1e+9*ns(:,2,1)-skew(v(16:18,1))*I_a2_O2*v(16:18,1))) + skew(v(16:18,1))*I_rot(:,7:9)*v(16:18,1);
%     wrench.ext(10:12,1) = Torq_a1;
%     wrench.ext(16:18,1) = Torq_a2;
    wrench.ext(11,1) = torque*1e+9;
    wrench.ext(17,1) = -torque*1e+9;

    %% Construct the Jacobian matrices

    % Bilateral constraints

    % Jacobian matrices
    % O1 B(1)/alpha1(2)
    J_O1_B         = [  eye(3)             zeros(3,3)  ; zeros(1,3)  (skew(ns(:,1,1))*ns(:,2,2))'  ; zeros(1,3)  (skew(ns(:,3,1))*ns(:,2,2))'];
    J_O1_a1        = [ -eye(3)  skew(vector.alpha1_O1) ; zeros(1,3) -(skew(ns(:,1,1))*ns(:,2,2))'  ; zeros(1,3) -(skew(ns(:,3,1))*ns(:,2,2))'];
 
    % O2 B(1)/alpha2(3)
    J_O2_B         = [  eye(3)              zeros(3,3)   ; zeros(1,3)   (skew(ns(:,1,1))*ns(:,2,3))'  ; zeros(1,3)   (skew(ns(:,3,1))*ns(:,2,3))'];
    J_O2_a2        = [ -eye(3)  skew(vector.alpha2_O2)   ; zeros(1,3)  -(skew(ns(:,1,1))*ns(:,2,3))'  ; zeros(1,3)  -(skew(ns(:,3,1))*ns(:,2,3))'];

    % A1 alpha1(2)/gamma1(4)
    J_A1_a1        = [  eye(3)  -skew(vector.alpha1_A1)   ; zeros(1,3)   (skew(ns(:,1,2))*ns(:,2,4))'  ; zeros(1,3)  (skew(ns(:,3,2))*ns(:,2,4))'];
    J_A1_g1        = [ -eye(3)   skew(vector.gamma1_A1)   ; zeros(1,3)  -(skew(ns(:,1,2))*ns(:,2,4))'  ; zeros(1,3) -(skew(ns(:,3,2))*ns(:,2,4))'];

    % A2 alpha2(3)/gamma2(5)
    J_A2_a2        = [  eye(3)  -skew(vector.alpha2_A2)   ; zeros(1,3)   (skew(ns(:,1,3))*ns(:,2,5))'  ; zeros(1,3)   (skew(ns(:,3,3))*ns(:,2,5))'];
    J_A2_g2        = [ -eye(3)   skew(vector.gamma2_A2)   ; zeros(1,3)  -(skew(ns(:,1,3))*ns(:,2,5))'  ; zeros(1,3)  -(skew(ns(:,3,3))*ns(:,2,5))'];

    % E gamma1(4)/gamma2(5)
    J_E_g1         = [  eye(3)  -skew(vector.gamma1_E)    ; zeros(1,3)   (skew(ns(:,1,4))*ns(:,2,5))'  ; zeros(1,3)   (skew(ns(:,3,4))*ns(:,2,5))'];
    J_E_g2         = [ -eye(3)   skew(vector.gamma2_E)    ; zeros(1,3)  -(skew(ns(:,1,4))*ns(:,2,5))'  ; zeros(1,3)  -(skew(ns(:,3,4))*ns(:,2,5))'];

    % B1 alpha1(2)/beta1(6)
    J_B1_a1        = [  eye(3)  -skew(vector.alpha1_B1)   ; zeros(1,3)   (skew(ns(:,1,2))*ns(:,2,6))'  ; zeros(1,3)   (skew(ns(:,3,2))*ns(:,2,6))'];
    J_B1_b1        = [ -eye(3)   skew(vector.beta1_B1)    ; zeros(1,3)  -(skew(ns(:,1,2))*ns(:,2,6))'  ; zeros(1,3)  -(skew(ns(:,3,2))*ns(:,2,6))'];

    % B2 alpha2(3)/beta2(7)
    J_B2_a2        = [  eye(3)  -skew(vector.alpha2_B2)   ; zeros(1,3)   (skew(ns(:,1,3))*ns(:,2,7))'  ; zeros(1,3)   (skew(ns(:,3,3))*ns(:,2,7))'];
    J_B2_b2        = [ -eye(3)   skew(vector.beta2_B2)    ; zeros(1,3)  -(skew(ns(:,1,3))*ns(:,2,7))'  ; zeros(1,3)  -(skew(ns(:,3,3))*ns(:,2,7))'];

    % C1 beta1(6)/delta1(8)
    J_C1_b1        = [  eye(3)  -skew(vector.beta1_C1)    ; zeros(1,3)   (skew(ns(:,1,6))*ns(:,2,8))'  ; zeros(1,3)   (skew(ns(:,3,6))*ns(:,2,8))'];
    J_C1_d1        = [ -eye(3)   skew(vector.delta1_C1)   ; zeros(1,3)  -(skew(ns(:,1,6))*ns(:,2,8))'  ; zeros(1,3)  -(skew(ns(:,3,6))*ns(:,2,8))'];

    % C2 beta2(7)/delta2(9)
    J_C2_b2        = [  eye(3)  -skew(vector.beta2_C2)    ; zeros(1,3)   (skew(ns(:,1,7))*ns(:,2,9))'  ; zeros(1,3)   (skew(ns(:,3,7))*ns(:,2,9))'];
    J_C2_d2        = [ -eye(3)   skew(vector.delta2_C2)   ; zeros(1,3)  -(skew(ns(:,1,7))*ns(:,2,9))'  ; zeros(1,3)  -(skew(ns(:,3,7))*ns(:,2,9))'];

    % D1 gamma1(4)/delta1(8)
    J_D1_g1        = [  eye(3)  -skew(vector.gamma1_D1)   ; zeros(1,3)   (skew(ns(:,1,4))*ns(:,2,8))'  ; zeros(1,3)   (skew(ns(:,3,4))*ns(:,2,8))'];
    J_D1_d1        = [ -eye(3)   skew(vector.delta1_D1)   ; zeros(1,3)  -(skew(ns(:,1,4))*ns(:,2,8))'  ; zeros(1,3)  -(skew(ns(:,3,4))*ns(:,2,8))'];

    % D2 gamma2(5)/delta2(9)
    J_D2_g2        = [  eye(3)  -skew(vector.gamma2_D2)   ; zeros(1,3)   (skew(ns(:,1,5))*ns(:,2,9))'  ; zeros(1,3)   (skew(ns(:,3,5))*ns(:,2,9))'];
    J_D2_d2        = [ -eye(3)   skew(vector.delta2_D2)   ; zeros(1,3)  -(skew(ns(:,1,5))*ns(:,2,9))'  ; zeros(1,3)  -(skew(ns(:,3,5))*ns(:,2,9))'];

    % F1 beta1(6)/epsilon1(10)
    J_F1_b1        = [  eye(3)  -skew(vector.beta1_F1)    ; zeros(1,3)   (skew(ns(:,1,6))*ns(:,2,10))' ; zeros(1,3)    (skew(ns(:,3,6))*ns(:,2,10))'];
    J_F1_e1        = [ -eye(3)   skew(vector.epsilon1_F1) ; zeros(1,3)  -(skew(ns(:,1,6))*ns(:,2,10))' ; zeros(1,3)   -(skew(ns(:,3,6))*ns(:,2,10))'];
    
    % F2 beta2(7)/epsilon2(11)
    J_F2_b2        = [  eye(3)  -skew(vector.beta2_F2)    ; zeros(1,3)   (skew(ns(:,1,7))*ns(:,2,11))' ; zeros(1,3)    (skew(ns(:,3,7))*ns(:,2,11))'];
    J_F2_e2        = [ -eye(3)   skew(vector.epsilon2_F2) ; zeros(1,3)  -(skew(ns(:,1,7))*ns(:,2,11))' ; zeros(1,3)   -(skew(ns(:,3,7))*ns(:,2,11))'];

    % G epsilon1(10)/epsilon2(11)
    J_G_e1         = [  eye(3)  -skew(vector.epsilon1_G)  ; zeros(1,3)   (skew(ns(:,1,10))*ns(:,2,11))' ; zeros(1,3)   (skew(ns(:,3,10))*ns(:,2,11))'];
    J_G_e2         = [ -eye(3)   skew(vector.epsilon2_G)  ; zeros(1,3)  -(skew(ns(:,1,10))*ns(:,2,11))' ; zeros(1,3)  -(skew(ns(:,3,10))*ns(:,2,11))'];

    % Restrict Rotation
    % O1 B/alpha1
    Jr_O1_B          = [zeros(1,3)   ns(:,2,1)'];
    Jr_O1_a1         = [zeros(1,3)  -ns(:,2,2)'];

    % O2 B/alpha2
    Jr_O2_B          = [zeros(1,3)   ns(:,2,1)'];
    Jr_O2_a2         = [zeros(1,3)  -ns(:,2,3)'];

    %% BASE CONSTRAINTS
    % fix BASE(B) to O
    J_BASE_O = [eye(3) -skew(BASE.BASE_O) zeros(3,(n-1)*6) ; zeros(3,3) eye(3) zeros(3,(n-1)*6)];

    % Restrict BASE(B) to vertical movement only(sliding)
    % moving axis: z-dir
    Js_BASE_trans = [[1;0;0]'   (skew(BASE.anchor)*[1;0;0])' ; [0;1;0]'   (skew(BASE.anchor)*[0;1;0])'];
%     Js_BASE_trans = [[1;0;0]'   1/2*(skew(-s(1:3,1))*[1;0;0])' ; [0;1;0]'   1/2*(skew(-s(1:3,1))*[0;1;0])']; % Pickl_MT_2009
    Js_BASE_rot   = [zeros(3,3)   eye(3)];
    Js_BASE = [Js_BASE_trans ; Js_BASE_rot];

    %% Bilateral Jacobian
    X = zeros(5, 6);
    x = zeros(1, 6);
%     z = zeros(1, 6);

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2, epsilon1,2(fixed)
    Jb = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X         X          X;
          J_O2_B         X   J_O2_a2        X        X         X        X         X        X         X          X;
               X   J_A1_a1         X  J_A1_g1        X         X        X         X        X         X          X;
               X         X   J_A2_a2        X  J_A2_g2         X        X         X        X         X          X;
               X         X         X   J_E_g1   J_E_g2         X        X         X        X         X          X;
               X   J_B1_a1         X        X        X   J_B1_b1        X         X        X         X          X;
               X         X   J_B2_a2        X        X         X  J_B2_b2         X        X         X          X;
               X         X         X        X        X   J_C1_b1        X   J_C1_d1        X         X          X;
               X         X         X        X        X         X  J_C2_b2         X  J_C2_d2         X          X;
               X         X         X  J_D1_g1        X         X        X   J_D1_d1        X         X          X;
               X         X         X        X  J_D2_g2         X        X         X  J_D2_d2         X          X;
               X         X         X        X        X   J_F1_b1        X         X        X   J_F1_e1          X;
               X         X         X        X        X         X  J_F2_b2         X        X         X    J_F2_e2;
               X         X         X        X        X         X        X         X        X    J_G_e1     J_G_e2;
         Js_BASE         X         X        X        X         X        X         X        X         X          X;
%          Jr_O1_B  Jr_O1_a1         x        x        x         x        x         x        x         x          x;
%          Jr_O2_B  Jr_O2_a2         x        x        x         x        x         x        x         x          x;
         ];       

    w0b = zeros(5*nc+no+n_fix,1);

    rank(Jb)

    %% Unilateral Constraints
    %  End effect CONSTRAINTS (contact constraint)
    
    % wn: difference between velocity components in constraint-space and the target velocities of the constraints functions
    % w0n: target velocities can be used to obtain a nonzero relative
    % velocities between the bodies(assume to be zero)
    
    contact_normal = [0 0 1]'; % z-dir
    % epsilon1(10)
    Jc_G_e1 = [contact_normal'  (skew(vector.epsilon1_G)*contact_normal)'];

    % epsilon2(11)
    Jc_G_e2 = [contact_normal'  (skew(vector.epsilon2_G)*contact_normal)'];
    
%     % Jacobian
%     Jn = [x    x    x    x    x    x    x    x    x       Jc_G_e1             x;
%           x    x    x    x    x    x    x    x    x             x       Jc_G_e2];
% 
%     % wn
%     wn = Jn*v;
%     w0n = [0;0];

    % Jacobian
    Jn = [x    x    x    x    x    x    x    x    x       Jc_G_e1             x];

    % wn
    wn = Jn*v;
    w0n = 0;    


    %% Constraint violation(error)
    % Phi: constraints in position-level
    % Linkage Order
    %  Link             x      q      
    % (1)Body         1:3    4:7
    % (2)alpha1      8:10  11:14
    % (3)alpha2     15:17  18:21
    % (4)gamma1     22:24  25:28
    % (5)gamma2     29:31  32:35
    % (6)beta1      36:38  39:42
    % (7)beta2      43:45  46:49
    % (8)delta1     50:52  53:56
    % (9)delta2     57:59  60:63
    % (10)epsilon1  64:66  67:70
    % (11)epsilon2  71:73  74:77

    % O1 B(1)/alpha1(2)
    Phi.O1_trans = s(8:10,1) + vector.alpha1_O1 - s(1:3,1) - BASE.BASE_O; % translation
    Phi.O1_rot = [ns(:,1,1)'*ns(:,2,2) ; ns(:,3,1)'*ns(:,2,2)]; % rotation

    % O2 B(1)/alpha2(3)
    Phi.O2_trans = s(15:17,1) + vector.alpha2_O2 - s(1:3,1)  - BASE.BASE_O;
    Phi.O2_rot = [ns(:,1,1)'*ns(:,2,3) ; ns(:,3,1)'*ns(:,2,3)];

    % A1 alpha1(2)/gamma1(4)
    Phi.A1_trans = s(22:24,1) + vector.gamma1_A1 - s(8:10,1) - vector.alpha1_A1;
    Phi.A1_rot = [ns(:,1,2)'*ns(:,2,4) ; ns(:,3,2)'*ns(:,2,4)];

    % A2 alpha2(3)/gamma2(5)
    Phi.A2_trans = s(29:31,1) + vector.gamma2_A2 - s(15:17,1) - vector.alpha2_A2;
    Phi.A2_rot = [ns(:,1,3)'*ns(:,2,5) ; ns(:,3,3)'*ns(:,2,5)];

    % E gamma1(4)/gamma2(5)
    Phi.E_trans  = s(29:31,1) + vector.gamma2_E  - s(22:24,1) - vector.gamma1_E;
    Phi.E_rot = [ns(:,1,4)'*ns(:,2,5) ; ns(:,3,4)'*ns(:,2,5)];

    % B1 alpha1(2)/beta1(6)
    Phi.B1_trans = s(36:38,1) + vector.beta1_B1 - s(8:10,1) - vector.alpha1_B1;
    Phi.B1_rot = [ns(:,1,2)'*ns(:,2,6) ; ns(:,3,2)'*ns(:,2,6)];

    % B2 alpha2(3)/beta2(7)
    Phi.B2_trans = s(43:45,1) + vector.beta2_B2 - s(15:17,1) - vector.alpha2_B2;
    Phi.B2_rot = [ns(:,1,3)'*ns(:,2,7) ; ns(:,3,3)'*ns(:,2,7)];

    % C1 beta1(6)/delta1(8)
    Phi.C1_trans = s(50:52,1) + vector.delta1_C1 - s(36:38,1) - vector.beta1_C1;
    Phi.C1_rot = [ns(:,1,6)'*ns(:,2,8) ; ns(:,3,6)'*ns(:,2,8)];

    % C2 beta2(7)/delta2(9)
    Phi.C2_trans = s(57:59,1) + vector.delta2_C2 - s(43:45,1) - vector.beta2_C2;
    Phi.C2_rot = [ns(:,1,7)'*ns(:,2,9) ; ns(:,3,7)'*ns(:,2,9)];

    % D1 gamma1(4)/delta1(8)
    Phi.D1_trans = s(50:52,1) + vector.delta1_D1 - s(22:24,1) - vector.gamma1_D1;
    Phi.D1_rot = [ns(:,1,4)'*ns(:,2,8) ; ns(:,3,4)'*ns(:,2,8)];

    % D2 gamma2(5)/delta2(9)
    Phi.D2_trans = s(57:59,1) + vector.delta2_D2 - s(29:31,1) - vector.gamma2_D2;
    Phi.D2_rot = [ns(:,1,5)'*ns(:,2,9) ; ns(:,3,5)'*ns(:,2,9)];

    % F1 beta1(6)/epsilon1(10)
    Phi.F1_trans = s(64:66,1) + vector.epsilon1_F1 - s(36:38,1) - vector.beta1_F1;
    Phi.F1_rot = [ns(:,1,6)'*ns(:,2,10) ; ns(:,3,6)'*ns(:,2,10)];

    % F2 beta2(7)/epsilon2(11)
    Phi.F2_trans = s(71:73,1) + vector.epsilon2_F2 - s(43:45,1) - vector.beta2_F2;
    Phi.F2_rot = [ns(:,1,7)'*ns(:,2,11) ; ns(:,3,7)'*ns(:,2,11)];

    % G epsilon1(10)/epsilon2(11)
    Phi.G_trans = s(71:73,1) + vector.epsilon2_G - s(64:66,1) - vector.epsilon1_G;
    Phi.G_rot = [ns(:,1,10)'*ns(:,2,11) ; ns(:,3,10)'*ns(:,2,11)];

    % BASE slider joint
%     Phi.sBASE_trans = [(s(1:3)+BASE.anchor)'*[1;0;0] ; (s(1:3)+BASE.anchor)'*[0;1;0]];
    Phi.sBASE_trans = [(s(1:3,1)+BASE.anchor)'*[1;0;0] ; (s(1:3,1)+BASE.anchor)'*[0;1;0]];
    Phi.sBASE_rot = quat2eul(s(4:7,1)')' - [0;0;0];

    % Restrict rotation
    ta1 = quat2eul(s(11:14,1)')';
    to = quat2eul(s(4:7,1)')';
    ta2 = quat2eul(s(18:21,1)')';
    Phi.rO_alpha1 = ta1(2) - to(2);
    Phi.rO_alpha2 = ta2(2) - to(2);

    Phi.bi_ERR = [Phi.O1_trans ; Phi.O1_rot    ; Phi.O2_trans ; Phi.O2_rot ; 
               Phi.A1_trans    ; Phi.A1_rot    ; Phi.A2_trans ; Phi.A2_rot ; 
               Phi.E_trans     ; Phi.E_rot     ;
               Phi.B1_trans    ; Phi.B1_rot    ; Phi.B2_trans ; Phi.B2_rot ;
               Phi.C1_trans    ; Phi.C1_rot    ; Phi.C2_trans ; Phi.C2_rot ; 
               Phi.D1_trans    ; Phi.D1_rot    ; Phi.D2_trans ; Phi.D2_rot ;
               Phi.F1_trans    ; Phi.F1_rot    ; Phi.F2_trans ; Phi.F2_rot ; 
               Phi.G_trans     ; Phi.G_rot     ; 
               Phi.sBASE_trans ; Phi.sBASE_rot ;
%                Phi.rO_alpha1   ; Phi.rO_alpha2
               ];
    
    % epsilon1 and GND (contact)
    Phi.epsilon1_contact = (s(64:66) + vector.epsilon1_G)'*contact_normal;

    % epsilon2 and GND (contact)
    Phi.epsilon2_contact = (s(71:73) + vector.epsilon2_G)'*contact_normal;

    Phi.uni_ERR = [Phi.epsilon1_contact ; Phi.epsilon2_contact];

    %% Regulartization
    % Unit transform
    % 1 N/m = 10^3 g mm/s^2 /mm;

    % The parameter is selected with reference to Numerical Solution of Mixed Linear Complementarity Problems in Multibody Dynamics with contact
    % bilateral
    para.bi.k = 1e+10 * 1e+3; % stiffness
    para.bi.b = 1e+8 * 1e+3; % damping
    
    % unilateral
    para.uni.k = 1e+5 * 1e+3; % stiffness
    para.uni.b = 1e+4 * 1e+3; % damping

    % stiffness matrix K
    para.bi.K = para.bi.k*eye(size(Jb,1));
    para.bi.B = para.bi.b*eye(size(Jb,1));
    % weighting
    biK_weight = [   1  1    1  0.75  0.75  , ...  % O1 B(1)/alpha1(2)
                     1  1    1  0.75  0.75  , ...  % O2 B(1)/alpha2(3)
                     1  1    1  0.75  0.75  , ...  % A1 alpha1(2)/gamma1(4)
                     1  1    1  0.75  0.75  , ...  % A2 alpha2(3)/gamma2(5)
                     1  1    1  0.75  0.75  , ...  % E gamma1(4)/gamma2(5)
                     1  1    1  0.75  0.75  , ...  % B1 alpha1(2)/beta1(6)
                     1  1    1  0.75  0.75  , ...  % B2 alpha2(3)/beta2(7)
                     1  1    1  0.75  0.75  , ...  % C1 beta1(6)/delta1(8)
                     1  1    1  0.75  0.75  , ...  % C2 beta2(7)/delta2(9)
                     1  1    1  0.75  0.75  , ...  % D1 gamma1(4)/delta1(8)
                     1  1    1  0.75  0.75  , ...  % D2 gamma2(5)/delta2(9)
                     1  1    1  0.75  0.75  , ...  % F1 beta1(6)/epsilon1(10)
                     1  1    1  0.75  0.75  , ...  % F2 beta2(7)/epsilon2(11)
                     1  1    1  0.75  0.75  , ...  % G epsilon1(10)/epsilon2(11)
                     1  1  1.5   1.5   1.5  , ...  % BASE slider joint
                     1  1                   , ...  % Restrict rotation
    ];
    para.bi.K_weight = diag(biK_weight);
%     para.bi.K = para.bi.K_weight*para.bi.K;
    

    % damping matrix B
    para.uni.K = para.uni.k*eye(size(Jn,1));
    para.uni.B = para.uni.b*eye(size(Jn,1));

    % matrix C
    % C = (h*B+h^2*K)^-1
    para.bi.C = inv(time.period * para.bi.B + time.period^2 * para.bi.K);
    para.uni.C = inv(time.period * para.uni.B + time.period^2 * para.uni.K);

    % vector d
    % d = (h*eye() + BK^-1)^-1*Phi;
    % inverse matrix calculation A\b for inv(A)*b or b/A for b*inv(A)
    para.bi.d = (time.period * eye(size(Jb,1)) + para.bi.B/para.bi.K)\Phi.bi_ERR;
    para.uni.d = (time.period * eye(size(Jn,1)) + para.uni.B/para.uni.K)\Phi.uni_ERR;

    %% Linear Complementary Problem
%     H = [MassMatrix  Jb' ; Jb  para.bi.C];
    H = [MassMatrix  Jb' ; Jb  zeros(size(Jb,1),size(Jb,1))];
    
    if(rank(H) ~= size(H,2))
        disp('Rank deficient');
    else
        disp('Full Rank');
    end

    G = [Jn  zeros(size(Jn,1),size(Jb,1))];
%     Y = para.uni.C;
    Y = zeros(size(Jn,1),size(Jn,1));
    b_bar = -MassMatrix*v + time.period * wrench.w - time.period * (wrench.ext+wrench.g);
%     b_bar = -MassMatrix*v + time.period * (wrench.ext+wrench.g);
%     by = [b_bar ; para.bi.d - w0b];
    by = [b_bar ; -w0b];
%     bx = para.uni.d - w0n;
    bx = -w0n;

    % linear complementarity problem(Ax+b = w, x>=0, w>=0, x'w=0)
    LCP.A = G*(H\G') + Y;
    LCP.b = -G*(H\by) + bx;

    [LCP.w,LCP.x,retcode] = LCPSolve(LCP.A,LCP.b);

    y = H\(G'*LCP.x - by);

    %% Calculate Reactional force
    wrench.react = (Jb'*y(6*n+1:end,1) + Jn'*LCP.x)/time.period;
    wrench.total = wrench.ext + wrench.g + wrench.react;
   
    Fn = Jn'*LCP.x/time.period;
    Fn = (Fn(57,1) + Fn(63,1))*1e-6;
    Fg = sum(wrench.g);

    %% Refresh system states
    v = y(1:6*n,1);

    for i = 0 : n-1
        % Update COM position
        s(i*7+1 : i*7+3) = s(i*7+1 : i*7+3) + time.period * v(i*6+1 : i*6+3);

        % Update quaternion (quaternion multiplication make sure quaternion do not exceed 1)
        [axis_,rot_] = Vec_to_Axis_Deg(v(i*6+4 : i*6+6));
        s(i*7+4 : i*7+7) = skew([cos(time.period*rot_/2);sin(time.period*rot_/2)*axis_])*s(i*7+4 : i*7+7);
    end

    %% set limit(theta can only be in the range of 17 to 160deg
    % angle between line OG and alpha1,2
    % if the angle is not in 17~160 then set s fixed to the boundary and
    % set v to zero
    OG = s(1:3) - (s(64:66,1)+vector.epsilon1_G);
    OGtheta = Angle_Between_2D_Vectors([OG(1);OG(3)],[0;1]);
    Rot = [cos(OGtheta) -sin(OGtheta) ; sin(OGtheta) cos(OGtheta)];
    a1theta_deg = Angle_Between_2D_Vectors([vector.alpha1_A1(1);vector.alpha1_A1(3)],Rot*[0;1])*180/pi;
    
    if(a1theta_deg >= 160-0.1 || a1theta_deg <= 17-0.1)
        disp('theta is no valid');
        break;
    end

    % s(1:3) = s(1:3) + time.period * v(1:3);
    % s(8:10) = s(8:10) + time.period * v(8:10);
    % s(15:18) = s(15:18) + time.period * v(14:16);
    
    % [axis1,rot1] = Vec_to_Axis_Deg(v(4:6));
    % s(4:7) = skew([cos(time.period*rot1/2);sin(time.period*rot1/2)*axis1])*s(4:7);
    % [axis2,rot2] = Vec_to_Axis_Deg(v(10:12));
    % s(11:14) = skew([cos(time.period*rot2/2);sin(time.period*rot2/2)*axis2])*s(11:14);


    %% Refresh points and vectors pointing from CoM to Hinge Joints

    % Linkage alpha1
    vector.alpha1_O1 = Rq(s(11:14))*alpha1.alpha1_O1;
    vector.alpha1_A1 = Rq(s(11:14))*alpha1.alpha1_A1;
    vector.alpha1_B1 = Rq(s(11:14))*alpha1.alpha1_B1;

    % Linkage alpha2
    vector.alpha2_O2 = Rq(s(18:21))*alpha2.alpha2_O2;
    vector.alpha2_A2 = Rq(s(18:21))*alpha2.alpha2_A2;
    vector.alpha2_B2 = Rq(s(18:21))*alpha2.alpha2_B2;

    % Linkage gamma1
    vector.gamma1_A1 = Rq(s(25:28))*gamma1.gamma1_A1;
    vector.gamma1_D1 = Rq(s(25:28))*gamma1.gamma1_D1;
    vector.gamma1_E  = Rq(s(25:28))*gamma1.gamma1_E;

    % Linkage gamma2
    vector.gamma2_A2 = Rq(s(32:35))*gamma2.gamma2_A2;
    vector.gamma2_D2 = Rq(s(32:35))*gamma2.gamma2_D2;
    vector.gamma2_E  = Rq(s(32:35))*gamma2.gamma2_E;

    % Linkage beta1
    vector.beta1_B1 = Rq(s(39:42))*beta1.beta1_B1;
    vector.beta1_C1 = Rq(s(39:42))*beta1.beta1_C1;
    vector.beta1_F1 = Rq(s(39:42))*beta1.beta1_F1;

    % Linkage beta2
    vector.beta2_B2 = Rq(s(46:49))*beta2.beta2_B2;
    vector.beta2_C2 = Rq(s(46:49))*beta2.beta2_C2;
    vector.beta2_F2 = Rq(s(46:49))*beta2.beta2_F2;

    % Linkage delta1
    vector.delta1_C1 = Rq(s(53:56))*delta1.delta1_C1;
    vector.delta1_D1 = Rq(s(53:56))*delta1.delta1_D1;

    % Linkage delta2
    vector.delta2_C2 = Rq(s(60:63))*delta2.delta2_C2;
    vector.delta2_D2 = Rq(s(60:63))*delta2.delta2_D2;

    % Linkage epsilon1
    vector.epsilon1_F1 = Rq(s(67:70))*epsilon1.epsilon1_F1;
    vector.epsilon1_G  = Rq(s(67:70))*epsilon1.epsilon1_G;

    % Linkage epsilon2
    vector.epsilon2_F2 = Rq(s(74:77))*epsilon2.epsilon2_F2;
    vector.epsilon2_G  = Rq(s(74:77))*epsilon2.epsilon2_G;

    toc
    %% Update book-keeping list
    history.s(:,t+1) = s;
    history.v(:,t+1) = v;
    history.J(:,:,t+1) = Jb;
    history.lambda(:,t+1) = lambda;
    history.wrench.ext(:,t+1) = wrench.ext;
    history.wrench.w(:,t+1) = wrench.w;
    history.wrench.g(:,t+1) = wrench.g;
    history.wrench.react(:,t+1) = wrench.react;
    history.wrench.total(:,t+1) = wrench.total;
    history.Fn(t+1,1) = Fn;
    history.theta(t+1,1) = a1theta_deg;
    history.O(t+1,:) = s(1:3);
    history.Pinertial(t+1,1) = v'*(MassMatrix*(v-history.v(:,t))/time.period);
    history.Pg(t+1,1) = v'*wrench.g;
    history.PFn(t+1,1) = v'*(Jn'*LCP.x/time.period);
    history.PFext(t+1,1) = v'*wrench.ext;
    history.Pinertial_BASE(t+1,1) = v(1:6,1)'*(MassMatrix(1:6,1:6)*(v(1:6,1)-history.v(1:6,t))/time.period);
    history.Rmasscenter(t+1,:) = (alpha1.m*s(8:10,1) + gamma1.m*s(22:24,1) + beta1.m*s(36:38,1) + delta1.m*s(50:52,1) + epsilon1.m*s(64:66,1))/(alpha1.m + gamma1.m + beta1.m + delta1.m + epsilon1.m);
    history.Lmasscenter(t+1,:) = (alpha2.m*s(15:17,1) + gamma2.m*s(29:31,1) + beta2.m*s(43:45,1) + delta2.m*s(57:59,1) + epsilon2.m*s(71:73,1))/(alpha2.m + gamma2.m + beta2.m + delta2.m + epsilon2.m);
    history.total_masscenter(t+1,:) = (BASE.m*(s(1:3,1)) + alpha1.m*s(8:10,1) + gamma1.m*s(22:24,1) + beta1.m*s(36:38,1) + delta1.m*s(50:52,1) + epsilon1.m*s(64:66,1) + alpha2.m*s(15:17,1) + gamma2.m*s(29:31,1) + beta2.m*s(43:45,1) + delta2.m*s(57:59,1) + epsilon2.m*s(71:73,1))/(sum(M));
%     temp1 = v'*(MassMatrix*(v-history.v(:,t))/time.period);
%     temp2 = v'*(Jn'*LCP.x/time.period);
%     temp3 = v'*wrench.g;
%     temp4 = v'*wrench.ext;
%     temp5 = temp1-temp2-temp3-temp4;
%     temp6 = v'*Jb'*y(6*n+1:end,1);
%     temp7 = v'*wrench.w;

    %% Plot
    if(PLOT_ANIMATION)
    %     subplot(2,2,[1,3]);
        PlotCuboid1(s(1:3,1),BASE.w,BASE.d,BASE.h,Rq(s(4:7,1)),'#EAC100','#977C00'); hold on
    %     plot3(s(1,1),s(2,1),s(3,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','k','MarkerEdgeColor','none'); hold on
        % linkage alpha1
        plot3(s(8,1),s(9,1),s(10,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','r','MarkerEdgeColor','none');
        plot3([s(1,1) s(8,1)+vector.alpha1_B1(1)],[s(2,1) s(9,1)+vector.alpha1_B1(2)],[s(3,1) s(10,1)+vector.alpha1_B1(3)],'LineWidth',2,'Color','r');
        % linkage alpha2
        plot3(s(15,1),s(16,1),s(17,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#EDB120','MarkerEdgeColor','none');
        plot3([s(1,1) s(15,1)+vector.alpha2_B2(1)],[s(2,1) s(16,1)+vector.alpha2_B2(2)],[s(3,1) s(17,1)+vector.alpha2_B2(3)],'LineWidth',2,'Color','#EDB120');
    
        % linkage gamma1
        plot3(s(22,1),s(23,1),s(24,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#D95319','MarkerEdgeColor','none');
        plot3([s(22,1)+vector.gamma1_E(1) s(22,1)+vector.gamma1_A1(1)],[s(23,1)+vector.gamma1_E(2) s(23,1)+vector.gamma1_A1(2)],[s(24,1)+vector.gamma1_E(3) s(24,1)+vector.gamma1_A1(3)],'LineWidth',2,'Color','#D95319');
        % linkage gamma2
        plot3(s(29,1),s(30,1),s(31,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#0072BD','MarkerEdgeColor','none');
        plot3([s(29,1)+vector.gamma2_E(1) s(29,1)+vector.gamma2_A2(1)],[s(30,1)+vector.gamma2_E(2) s(30,1)+vector.gamma2_A2(2)],[s(31,1)+vector.gamma2_E(3) s(31,1)+vector.gamma2_A2(3)],'LineWidth',2,'Color','#0072BD');
    
        % linkage beta1
        plot3(s(36,1),s(37,1),s(38,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#0000C6','MarkerEdgeColor','none');
    %     plot3([s(36,1)+vector.beta1_B1(1) s(36,1)+vector.beta1_C1(1)],[s(37,1)+vector.beta1_B1(2) s(37,1)+vector.beta1_C1(2)],[s(38,1)+vector.beta1_B1(3) s(38,1)+vector.beta1_C1(3)],'LineWidth',2,'Color','#0000C6');
    %     plot3([s(36,1)+vector.beta1_C1(1) s(36,1)+vector.beta1_F1(1)],[s(37,1)+vector.beta1_C1(2) s(37,1)+vector.beta1_F1(2)],[s(38,1)+vector.beta1_C1(3) s(38,1)+vector.beta1_F1(3)],'LineWidth',2,'Color','#0000C6');
        % linkage beta2
        plot3(s(43,1),s(44,1),s(45,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#F75000','MarkerEdgeColor','none');
    %     plot3([s(43,1)+vector.beta2_B2(1) s(43,1)+vector.beta2_C2(1)],[s(44,1)+vector.beta2_B2(2) s(44,1)+vector.beta2_C2(2)],[s(45,1)+vector.beta2_B2(3) s(45,1)+vector.beta2_C2(3)],'LineWidth',2,'Color','#F75000');
    %     plot3([s(43,1)+vector.beta2_C2(1) s(43,1)+vector.beta2_F2(1)],[s(44,1)+vector.beta2_C2(2) s(44,1)+vector.beta2_F2(2)],[s(45,1)+vector.beta2_C2(3) s(45,1)+vector.beta2_F2(3)],'LineWidth',2,'Color','#F75000');
        % linkage delta1
        plot3(s(50,1),s(51,1),s(52,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#77AC30','MarkerEdgeColor','none');
        plot3([s(50,1)+vector.delta1_C1(1) s(50,1)+vector.delta1_D1(1)],[s(51,1)+vector.delta1_C1(2) s(51,1)+vector.delta1_D1(2)],[s(52,1)+vector.delta1_C1(3) s(52,1)+vector.delta1_D1(3)],'LineWidth',2,'Color','#77AC30');
        %linkage delta2
        plot3(s(57,1),s(58,1),s(59,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#A2142F','MarkerEdgeColor','none');
        plot3([s(57,1)+vector.delta2_C2(1) s(57,1)+vector.delta2_D2(1)],[s(58,1)+vector.delta2_C2(2) s(58,1)+vector.delta2_D2(2)],[s(59,1)+vector.delta2_C2(3) s(59,1)+vector.delta2_D2(3)],'LineWidth',2,'Color','#A2142F');
    
        % linkage epsilon1
        plot3(s(64,1),s(65,1),s(66,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#5B00AE','MarkerEdgeColor','none');
    %     plot3([s(64,1)+vector.epsilon1_F1(1) s(64,1)+vector.epsilon1_G(1)],[s(65,1)+vector.epsilon1_F1(2) s(65,1)+vector.epsilon1_G(2)],[s(66,1)+vector.epsilon1_F1(3) s(66,1)+vector.epsilon1_G(3)],'LineWidth',2,'Color','#5B00AE');
        % linkage epsilon2
        plot3(s(71,1),s(72,1),s(73,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','#007500','MarkerEdgeColor','none');
    %     plot3([s(71,1)+vector.epsilon2_F2(1) s(71,1)+vector.epsilon2_G(1)],[s(72,1)+vector.epsilon2_F2(2) s(72,1)+vector.epsilon2_G(2)],[s(73,1)+vector.epsilon2_F2(3) s(73,1)+vector.epsilon2_G(3)],'LineWidth',2,'Color','#007500');
    
        % right mass center
    %     plot3(history.Rmasscenter(t+1,1),history.Rmasscenter(t+1,2),history.Rmasscenter(t+1,3),'Marker','o','MarkerSize',5,'MarkerFaceColor','r','MarkerEdgeColor','none');
        % left mass center
    %     plot3(history.Lmasscenter(t+1,1),history.Lmasscenter(t+1,2),history.Lmasscenter(t+1,3),'Marker','o','MarkerSize',5,'MarkerFaceColor','b','MarkerEdgeColor','none');
    
        plot3(history.total_masscenter(t+1,1),history.total_masscenter(t+1,2),history.total_masscenter(t+1,3),'Marker','o','MarkerSize',5,'MarkerFaceColor','k','MarkerEdgeColor','none');
        
    
        [rupframe_3D,rdownframe_3D] = outframe_coordinate3D(s(36:38,1)+vector.beta1_B1, s(36:38,1)+vector.beta1_C1, s(36:38,1)+vector.beta1_F1, s(64:66,1)+vector.epsilon1_G,R,up_deg);
        plot3(rupframe_3D(:,1),rupframe_3D(:,2),rupframe_3D(:,3),'LineWidth',2,'Color','#0000C6');
        plot3(rdownframe_3D(:,1),rdownframe_3D(:,2),rdownframe_3D(:,3),'LineWidth',2,'Color','#5B00AE');
    
        [lupframe_3D,ldownframe_3D] = outframe_coordinate3D(s(43:45,1)+vector.beta2_B2, s(43:45,1)+vector.beta2_C2, s(43:45,1)+vector.beta2_F2, s(71:73,1)+vector.epsilon2_G,R,up_deg);
        plot3(lupframe_3D(:,1),lupframe_3D(:,2),lupframe_3D(:,3),'LineWidth',2,'Color','#F75000');
        plot3(ldownframe_3D(:,1),ldownframe_3D(:,2),ldownframe_3D(:,3),'LineWidth',2,'Color','#007500');
    
        plot3([-150 150],[0,0],[0 0],'Color','k','LineWidth',1.5);
    
       % plot forces
        arrow3d([s(71,1)+vector.epsilon2_G(1) s(71,1)+vector.epsilon2_G(1)],[s(72,1)+vector.epsilon2_G(2) s(72,1)+vector.epsilon2_G(2)],[s(73,1)+vector.epsilon2_G(3) s(73,1)+vector.epsilon2_G(3)+Fn],aa,bb,cc,'k');
    
        view(0,0);
        
        hold off
        axis equal
        axis([-150 150 -200 200 -10 420]);
        
        
    %     addpoints(h1,t*time.period,km*alignedshift_data(t+index.Motor_startVJ-1,4));
    %     addpoints(h2,t*time.period,km*alignedshift_data(t+index.Motor_startVJ-1,7));
    %     addpoints(h3,t*time.period,alignedshift_data(t+index.Motor_startVJ-1,8));
    %     addpoints(h4,t*time.period,-Fn);
    
        drawnow;

    end

    %% save to gif
%     frame = getframe(1);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     filename = 'MBD_withcontact.gif';
%     if t == 1
%         imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',del);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',del);
%     end

    % ----- save to video(avi) -----
%     frame = getframe(gcf);
%     writeVideo(writerObj, frame);
    
end

%% 
% plot3(history.Rmasscenter(:,1),history.Rmasscenter(:,2),history.Rmasscenter(:,3),'Marker','o','MarkerSize',2,'MarkerFaceColor','r','MarkerEdgeColor','none'); hold on
% plot3(history.Lmasscenter(:,1),history.Lmasscenter(:,2),history.Lmasscenter(:,3),'Marker','o','MarkerSize',2,'MarkerFaceColor','b','MarkerEdgeColor','none');
% plot3(history.total_masscenter(:,1),history.total_masscenter(:,2),history.total_masscenter(:,3),'Marker','o','MarkerSize',2,'MarkerFaceColor','b','MarkerEdgeColor','none');
% view(0,0);
% axis equal
% axis([-80 80 -200 200 80 180]);

%% plot
% figure
% theta0_deg = 17;
% 
% gcf_arr = [0.3,-0.1,0.3,1.2];
% set(gcf,'unit','normalized','position',gcf_arr);
% 
% time.T = (0:1:time.step)'*time.period;
% 
% % h(1) = subplot(6,1,1);
% subaxis(6,1,1,'MarginTop',0.025,'MarginRight',0.05);
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,2),'LineWidth',1.5,'DisplayName','Position command(R)','Color','b'); hold on
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,3),'LineWidth',1.5,'DisplayName','Encoder(R)','LineStyle','--','Color','#D95319');
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,5),'LineWidth',1.5,'DisplayName','Position command(L)','Color','#7E2F8E');
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,6),'LineWidth',1.5,'DisplayName','Encoder(L)','LineStyle','-.','Color','#77AC30');
% xline((index.Vicon_LO-index.Motor_startVJ)*time.period,'Color','#EDB120','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','Exp lift-off');
% hold off
% title('position-t','Interpreter','latex');
% xlabel('t[sec]','Interpreter','latex');
% ylabel('position[rad]','Interpreter','latex');
% xlim([0 time.step*time.period]);
% ylim([-0.1 2*pi+3.5]);
% legend('Location','northwest','Interpreter','latex','Orientation','horizontal','NumColumnsMode','manual','NumColumns',2);
% 
% % h(2) = subplot(6,1,2);
% subaxis(6,1,2,'MarginTop',0.025,'MarginRight',0.05);
% % plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.Vicon_LO,4),'Color','#77AC30','LineWidth',1.5,'DisplayName','Exp-current R'); hold on
% % plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.Vicon_LO,7),'Color','#7E2F8E','LineWidth',1.5,'DisplayName','Exp-current L');
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,4),'Color','#77AC30','LineWidth',1.5,'DisplayName','Exp-current R'); hold on
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,7),'LineStyle','-.','Color','#D95319','LineWidth',1.5,'DisplayName','Exp-current L');
% xline((index.Vicon_LO-index.Motor_startVJ)*time.period,'Color','#EDB120','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','Exp lift-off');
% hold off
% title('current-t','Interpreter','latex');
% xlabel('t[sec]','Interpreter','latex');
% ylabel('current[A]','Interpreter','latex');
% xlim([0 time.step*time.period]);
% ylim([-20 28]);
% legend('Location','northwest','Interpreter','latex','Orientation','horizontal');
% 
% % h(3) = subplot(6,1,3);
% subaxis(6,1,3,'MarginTop',0.025,'MarginRight',0.05);
% temp = alignedshift_data(:,11)-mean(alignedshift_data(1:300,9));
% temp(index.Vicon_LO:end,1) = nan;
% temp = temp(index.Motor_startVJ-1:index.end_plot);
% % plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.Vicon_LO,9)-mean(alignedshift_data(1:300,9)));
% % plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,9)-mean(alignedshift_data(1:300,9)),'LineWidth',1.5,'Color','#CE0000','DisplayName','posG,z');
% plot(time.T,history.O(:,3),'Color','b','LineWidth',1.5,'LineStyle','-.','DisplayName','MBD hipO,z'); hold on
% plot(time.T,temp,'Color','#CE0000','LineWidth',1.5,'DisplayName','Exp hipO,z');
% xline((t-1)*time.period,'Color','#4DBEEE','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','Label','MBD lift-off','HandleVisibility','off');
% xline((index.Vicon_LO-index.Motor_startVJ)*time.period,'Color','#EDB120','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','Label','Exp lift-off','HandleVisibility','off');
% % ylim([-10 alignedshift_data(index.end_plot,9)-mean(alignedshift_data(1:300,9))+25]);
% hold off
% xlim([0 time.step*time.period]);
% title('hipO,z-t','Interpreter','latex');
% xlabel('t[sec]','Interpreter','latex');
% ylabel('hipO,z[mm]','Interpreter','latex');
% legend('Location','northwest','Interpreter','latex','Orientation','vertical');
% 
% % h(4) = subplot(6,1,4);
% subaxis(6,1,4,'MarginTop',0.025,'MarginRight',0.05);
% plot(time.T,history.theta,'Color','b','LineWidth',1.5,'LineStyle','-.','DisplayName','MBD theta'); hold on
% plot(time.T,theta,'Color','#CE0000','LineWidth',1.5,'DisplayName','Exp theta');
% xline((t-1)*time.period,'Color','#4DBEEE','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','MBD lift-off');
% xline((index.Vicon_LO-index.Motor_startVJ)*time.period,'Color','#EDB120','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','Exp lift-off');
% xlim([0 time.step*time.period]);
% title('$\theta$ -t','Interpreter','latex');
% xlabel('t[sec]','Interpreter','latex');
% ylabel('$\theta$ [deg]','Interpreter','latex');
% legend('Location','northwest','Interpreter','latex');
% 
% % h(5) = subplot(6,1,5);
% subaxis(6,1,5,'MarginTop',0.025,'MarginRight',0.05);
% plot(time.T,-history.Fn,'Color','b','LineWidth',1.5,'LineStyle','-.','DisplayName','GRF-MBD'); hold on
% % plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.Vicon_LO,8),'Color','#D95319','LineStyle','-','LineWidth',1.5,'DisplayName','Exp-Fn');
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,8),'Color','#CE0000','LineStyle','-','LineWidth',1.5,'DisplayName','GRF-Exp');
% % plot(time.T,Fn_vw,'Color','#77AC30','LineStyle','--','LineWidth',1.5,'DisplayName','Virtual Work-Fn,z');
% xline((t-1)*time.period,'Color','#4DBEEE','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','MBD lift-off');
% xline((index.Vicon_LO-index.Motor_startVJ)*time.period,'Color','#EDB120','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','Exp lift-off');
% xlim([0 time.step*time.period]);
% ylim([-max(history.Fn) 10]);
% title('GRF -t','Interpreter','latex');
% xlabel('t[sec]','Interpreter','latex');
% ylabel('GRF [N]','Interpreter','latex');
% legend('Location','northwest','Interpreter','latex','Orientation','horizontal');
% 
% % h(6) = subplot(6,1,6);
% subaxis(6,1,6,'MarginTop',0.025,'MarginRight',0.05);
% plot(time.T,-history.Fn,'Color','b','LineWidth',1.5,'LineStyle','-.','DisplayName','GRF-MBD'); hold on
% % plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.Vicon_LO,8),'Color','#D95319','LineStyle','-','LineWidth',1.5,'DisplayName','Exp-Fn');
% plot(time.T,alignedshift_data(index.Motor_startVJ-1:index.end_plot,8),'Color','#CE0000','LineStyle','-','LineWidth',1.5,'DisplayName','GRF-Exp');
% plot(time.T,Fn_vw,'Color','#77AC30','LineStyle','--','LineWidth',1.5,'DisplayName','GRF-Virtual Work');
% xline((t-1)*time.period,'Color','#4DBEEE','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','MBD lift-off');
% xline((index.Vicon_LO-index.Motor_startVJ)*time.period,'Color','#EDB120','LineWidth',1.5,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off','Label','Exp lift-off');
% xlim([0 time.step*time.period]);
% title('GRF -t','Interpreter','latex');
% xlabel('t[sec]','Interpreter','latex');
% ylabel('GRF [N]','Interpreter','latex');
% legend('Location','northwest','Interpreter','latex','Orientation','horizontal');
% 
% % align legend blocks
% % m = zeros(length(h),4);
% % for k=1:length(h)
% %     m(k,:) = get(h(k),'Position');
% % end
% % 
% % m(:,3) = max(m(:,3))-0.2;
% % for k=1:length(h)
% %     set(h(k),'Position',m(k,:));
% % end
