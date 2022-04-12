clear all; close all; clc;
%% Calculating all the force/torque apples on each components of the linkage -wheel leg by constrained dynamics
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
time.step = 1;

% Useful Constants
n = 11; % Number of linkages

%Environment
g = 9810 ;      % Gravitaional acceleration

% Constraints
no = 6; % Number of constraints to fix BASE to O
nc = 14; % Number of joints(hinge)
n_fix = 0; % Number of constraints to restrict the rotation around y-axis between alpha1 and O, alpha2 and O

% Redundancy
nr = 12;

% plot
a = 0.7;
b = 0.7;
c = 3;

% gif
del = 0.01; % time between animation frames

% create the video writer with 1 fps
% writerObj = VideoWriter('myVideo_fixed.avi');
% writerObj.FrameRate = 100;
% set the seconds per image
% open the video writer
% open(writerObj);

%% Initial Conditions
% Computing points and angle between linkage and [1 0] according to initial theta
beta_deg = 0;
theta_deg = 17;
points = calculate_coordinate_flat2(theta_deg,beta_deg);
angle = linkleg_angle(points);

% load R100 parameter
load('param_R100.mat','R','up_deg');

%% Linkage configurations (Using metic units - mmgs)

% % Assume BASE is fixed to O(fix joint)
% % Mass BASE
% BASE.h = 100; % Height (Assume B is a solid cuboid)
% BASE.w = 100; % Width
% BASE.d = 100; % Depth
% BASE.x = [0 0 0]'; % Position of mass BASE
% BASE.BASE_O = [0 0 0]';
% BASE.theta = 0 ;
% BASE.q = [cos(BASE.theta/2) 0 0 sin(BASE.theta/2)]'; % Orientation of mass B (quaternion)
% BASE.m = 1000; % Mass
% BASE.I = BASE.m/12 * [ BASE.h^2+BASE.d^2 0 0 ; 0 BASE.w^2+BASE.d^2 0 ; 0 0 BASE.w^2+BASE.h^2 ]; % Inertia Tensor
% BASE.W = [(1/BASE.m)*eye(3) zeros(3,3);zeros(3,3) inv(BASE.I)]; % Inverse inertia matrix
% 
% % ---------- Linkage alpha-1(Rigid Body) ----------
% alpha1.m = 49.438724 ;
% alpha1.I = [5226.992514	 5575.651755	 30.511514 ;
% 	        5575.651755	 52731.315884	 2.550826 ;
% 	        30.511514	 2.550826	     51270.275795 ];
% 
% % alpha1.I = [ 5226.992514	 5575.651755	 -30.511514 ;
% % 	         5575.651755	 52731.315884	   2.550826 ;
% % 	          -30.511514	 2.550826	   51270.275795 ];
% 
% alpha1.W = [(1/alpha1.m)*eye(3) zeros(3,3);zeros(3,3) inv(alpha1.I)]; % Inverse inertia matrix
% alpha1.q = [cos(angle.alpha1_phi/2) 0 -sin(angle.alpha1_phi/2) 0]';
% 
% % alpha1.alpha1_O1 = [-36.45 0 0.02]'; % [-36.45 -8.63 0.02]'; % The vector pointing from the C.O.M of alpha-1 to O in the local frame.
% alpha1.alpha1_O1 = [-36.45 -8.63 0.02]'; % local frame
% alpha1.len = norm(alpha1.alpha1_O1);
% alpha1.x = Rq(BASE.q)*BASE.x - Rq(alpha1.q)*alpha1.alpha1_O1; % global frame
% alpha1.alpha1_A1 = Rq(alpha1.q)'*(Rq(BASE.q)*points.A1_3d - alpha1.x); % local frame
% alpha1.alpha1_B1 = Rq(alpha1.q)'*(Rq(BASE.q)*points.B1_3d - alpha1.x); % local frame
% % --------------------------------------------------
% 
% % ---------- Linkage alpha-2(Rigid Body) -----------
% % alpha2.m = 49.438724 ;  %48.332901;
% alpha2.m = 48.332901;
% alpha2.I = [5226.992514	 5575.651755	 30.511514 ;
% 	        5575.651755	 52731.315884	 2.550826 ;
% 	        30.511514	 2.550826	     51270.275795 ]; 
% %             [  5320.371779	5878.366912      -26.243856;
% %         	 -5878.366912	53989.169452	   1.415240;
% % 	           -26.243856	    1.415240   52405.042766];
% 
% % alpha2.I = [  5320.372467   -5878.155734      26.348805;
% %              -5878.155734   53989.571406       1.355846;
% %                 26.348805       1.355846   52405.385266];
% alpha2.W = [(1/alpha2.m)*eye(3) zeros(3,3);zeros(3,3) inv(alpha2.I)];
% alpha2.q = [cos(angle.alpha2_phi/2) 0 -sin(angle.alpha2_phi/2) 0]';
% 
% % alpha2.alpha2_O2 = [-36.43   0   -0.02 ]';
% alpha2.alpha2_O2 = [-36.45 -8.63 -0.02]'; % local frame
% alpha2.len = norm(alpha2.alpha2_O2);
% alpha2.x = Rq(BASE.q)*BASE.x - Rq(alpha2.q)*alpha2.alpha2_O2; % global frame
% alpha2.alpha2_A2 = Rq(alpha2.q)'*(Rq(BASE.q)*points.A2_3d - alpha2.x); % local frame
% alpha2.alpha2_B2 = Rq(alpha2.q)'*(Rq(BASE.q)*points.B2_3d - alpha2.x); % local frame
% % ----------------------------------------------------
% 
% % ---------- Linkage gamma-1(Rigid Body) ----------
% gamma1.m =  49.049581 ;
% % gamma1.m =  0.001 ;
% gamma1.I =[ 2956.157949	    0.004740	  1300.105736 ;
% 	        0.004740	    92320.757630  -0.000375 ;
% 	        1300.105736	    -0.000375	  90182.095841];
%  
% gamma1.W = [(1/gamma1.m)*eye(3) zeros(3,3);zeros(3,3) inv(gamma1.I)];
% gamma1.q = [cos(angle.gamma1_phi/2) 0 -sin(angle.gamma1_phi/2) 0]';
% 
% gamma1.gamma1_A1 = [-65.68 0 -4.57]'; %[-65.68 5 -4.57]';
% % gamma1.gamma1_A1 = [-65.68 5 -4.57]';  % local frame
% gamma1.len = norm(gamma1.gamma1_A1);
% gamma1.x = Rq(BASE.q)*points.A1_3d - Rq(gamma1.q)*gamma1.gamma1_A1; % global frame
% gamma1.gamma1_D1 = Rq(gamma1.q)'*(Rq(BASE.q)*points.D1_3d - gamma1.x); % local frame
% gamma1.gamma1_E  = Rq(gamma1.q)'*(Rq(BASE.q)*points.E_3d - gamma1.x); % local frame
% % -------------------------------------------------
% 
% 
% % ---------- Linkage gamma-2(Rigid Body) ----------
% gamma2.m = 49.049581;
% % gamma2.m = 0.001;
% gamma2.I = [ 2956.157949	  -0.004740	   -1300.105736 ;
% 	        -0.004740	     92320.757630	  -0.000375 ;
% 	        -1300.105736	   -0.000375	  90182.095841];
% 
% gamma2.W = [(1/gamma2.m)*eye(3) zeros(3,3);zeros(3,3) inv(gamma2.I)];
% gamma2.q = [cos(angle.gamma2_phi/2) 0 -sin(angle.gamma2_phi/2) 0]';
% 
% gamma2.gamma2_A2 =  [-65.68 0 4.57]'; 
% % gamma2.gamma2_A2 =  [-65.68 -5 4.57]'; %local frame
% gamma2.len = norm(gamma2.gamma2_A2);
% gamma2.x = Rq(BASE.q)*points.A2_3d - Rq(gamma2.q)*gamma2.gamma2_A2;  % global frame
% gamma2.gamma2_D2 = Rq(gamma2.q)'*(Rq(BASE.q)*points.D2_3d - gamma2.x); % local frame
% gamma2.gamma2_E  = Rq(gamma2.q)'*(Rq(BASE.q)*points.E_3d - gamma2.x); % local frame
% % -------------------------------------------------
% 
% % ----------- Linkage beta-1(Rigid Body) ----------
% beta1.m = 142.168440;
% beta1.I = [ 91843.636529	 -772.772852	 40829.647339 ;
%             -772.772852    517848.015598	 -201.475342  ;
%         	40829.647339	 -201.475342	 493681.548715];
% 
% beta1.W = [(1/beta1.m)*eye(3) zeros(3,3);zeros(3,3) inv(beta1.I)];
% beta1.q = [cos(angle.beta1_phi/2) 0 -sin(angle.beta1_phi/2) 0]';
% 
% beta1.beta1_B1 = [-75.5 0 -28.66]'; % local frame
% beta1.x = Rq(BASE.q)*points.B1_3d - Rq(beta1.q)*beta1.beta1_B1; % global frame
% beta1.beta1_C1 = Rq(beta1.q)'*(Rq(BASE.q)*points.C1_3d - beta1.x); % local frame
% beta1.beta1_F1 = Rq(beta1.q)'*(Rq(BASE.q)*points.F1_3d - beta1.x); % local frame
% % -------------------------------------------------
% 
% % ----------- Linkage beta-2(Rigid Body) ----------
% beta2.m = 142.168440;
% beta2.I = [ 91843.636529	  772.772852	 -40829.647339 ;
%              772.772852    517848.015598	 -201.475342  ;
%         	-40829.647339	 -201.475342	 493681.548715];
% 
% beta2.W = [(1/beta2.m)*eye(3) zeros(3,3);zeros(3,3) inv(beta2.I)];
% beta2.q = [cos(angle.beta2_phi/2) 0 -sin(angle.beta2_phi/2) 0]';
% 
% beta2.beta2_B2 = [-75.5 0 28.66]'; % local frame
% beta2.x = Rq(BASE.q)*points.B2_3d - Rq(beta2.q)*beta2.beta2_B2; % global frame
% beta2.beta2_C2 = Rq(beta2.q)'*(Rq(BASE.q)*points.C2_3d - beta2.x); % local frame
% beta2.beta2_F2 = Rq(beta2.q)'*(Rq(BASE.q)*points.F2_3d - beta2.x); % local frame
% % -------------------------------------------------
% 
% % ---------- Linkage delta-1(Rigid Body) ----------
% delta1.m =  15.681471 ;
% delta1.I = [  352.062649	 0.000000	    0.000000;
% 	            0.000000	 13277.939609	 0.000000;
% 	            0.000000	 0.000000	 13187.234818];
%  
% delta1.W = [(1/delta1.m)*eye(3) zeros(3,3);zeros(3,3) inv(delta1.I)];
% delta1.q = [cos(angle.delta1_phi/2) 0 -sin(angle.delta1_phi/2) 0]';
% 
% delta1.delta1_D1 =  [-42.6 0 0]'; % [-42.6 5 0]';
% delta1.x = Rq(BASE.q)*points.D1_3d - Rq(delta1.q)*delta1.delta1_D1; % global frame
% delta1.delta1_C1 = Rq(delta1.q)'*(Rq(BASE.q)*points.C1_3d - delta1.x); % local frame
% % -------------------------------------------------
% 
% % ---------- Linkage delta-2(Rigid Body) ----------
% delta2.m =   15.681471;
% delta2.I = [  352.062649	 0.000000	    0.000000;
% 	            0.000000	 13277.939609	 0.000000;
% 	            0.000000	 0.000000	 13187.234818];
% 
% delta2.W = [(1/delta2.m)*eye(3) zeros(3,3);zeros(3,3) inv(delta2.I)];
% delta2.q = [cos(angle.delta2_phi/2) 0 -sin(angle.delta2_phi/2) 0]';
% 
% delta2.delta2_D2 =  [-42.6 0 0]'; % [-42.6 -5 0]'; 
% delta2.x = Rq(BASE.q)*points.D2_3d - Rq(delta2.q)*delta2.delta2_D2; % global frame
% delta2.delta2_C2 = Rq(delta2.q)'*(Rq(BASE.q)*points.C2_3d - delta2.x); % local frame
% % -------------------------------------------------
% 
% % ---------- Linkage epsilon-1(Rigid Body) ----------
% epsilon1.m =  66.713674;
% epsilon1.I = [10744.530879   1691.358784	   735.827120 ;
% 	           1691.358784	  57872.558479	  779.660039 ;
% 	            735.827120	  779.660039	  58548.216359];
% 
% epsilon1.W = [(1/epsilon1.m)*eye(3) zeros(3,3);zeros(3,3) inv(epsilon1.I)];
% epsilon1.q = [cos(angle.epsilon1_phi/2) 0 -sin(angle.epsilon1_phi/2) 0]';
% 
% epsilon1.epsilon1_F1 = [-44.1 0 -7.6]';
% epsilon1.x = Rq(BASE.q)*points.F1_3d - Rq(epsilon1.q)*epsilon1.epsilon1_F1; % global frame
% epsilon1.epsilon1_G = Rq(epsilon1.q)'*(Rq(BASE.q)*points.G_3d - epsilon1.x); % local frame
% % ---------------------------------------------------
% 
% % ---------- Linkage epsilon-2(Rigid Body) ----------
% epsilon2.m =  66.713674;
% epsilon2.I = [10744.530879    -1691.358784	   -735.827120 ;
% 	          -1691.358784	  57872.558479	    779.660039 ;
% 	           -735.827120	    779.660039	  58548.216359];
% 
% epsilon2.W = [(1/epsilon2.m)*eye(3) zeros(3,3);zeros(3,3) inv(epsilon2.I)];
% epsilon2.q = [cos(angle.epsilon2_phi/2) 0 -sin(angle.epsilon2_phi/2) 0]';
% 
% epsilon2.epsilon2_F2 = [-44.1 0 7.6]';
% epsilon2.x = Rq(BASE.q)*points.F2_3d - Rq(epsilon2.q)*epsilon2.epsilon2_F2; % global frame
% epsilon2.epsilon2_G = Rq(epsilon2.q)'*(Rq(BASE.q)*points.G_3d - epsilon2.x); % local frame
% % ---------------------------------------------------

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

% Combining the unit vectors for convenience
% ns = [BASE.nx ... alpha1.nx ... alpha2.nx ... beta1.nx ........epsilon2.nz]

ns_ = zeros(3,3,n);
for i = 1 : n
    ns_(:,:,i) = eye(3);  % [ nx ny nz ] = eye(3)
end

% M = [BASE.m alpha1.m alpha2.m];
% I = [BASE.I alpha1.I alpha2.I];
% M = [BASE.m alpha1.m alpha2.m gamma1.m gamma2.m];
% I = [BASE.I alpha1.I alpha2.I gamma1.I gamma2.I];
% M = [BASE.m alpha1.m alpha2.m gamma1.m gamma2.m beta1.m beta2.m delta1.m delta2.m];
% I = [BASE.I alpha1.I alpha2.I gamma1.I gamma2.I beta1.I beta2.I delta1.I delta2.I];
M = [BASE.m alpha1.m alpha2.m gamma1.m gamma2.m beta1.m beta2.m delta1.m delta2.m epsilon1.m epsilon2.m];
I = [BASE.I alpha1.I alpha2.I gamma1.I gamma2.I beta1.I beta2.I delta1.I delta2.I epsilon1.I epsilon2.I];

%% Book-keeping
history.s = zeros(7*n,time.step+1);
history.v = zeros(6*n,time.step+1);
history.J = zeros(5*nc+no+n_fix-nr,6*n,time.step+1);
history.lambda = zeros(5*nc+no+n_fix-nr,time.step+1);

%% Initialization
% system states: (linkage C.O.M position & quaternion in world frame)
% s = (BASE.x,BASE.q,alpha1.x,alpha1.q,...)
% velocity: (C.O.M velocities in world frame)
% v = (BASE.v,BASE.w,alpha1.v,alpha1.w,...)

% inverse mass matrix
W = zeros(6*n,6*n);

% externel wrench input
wrench.ext = zeros(6*n,1);

% non torque term of the euler's of motion
wrench.w = zeros(6*n,1);

% Gravitational force
wrench.g = zeros(6,n);
wrench.g(3,:) = M*(-g);
wrench.g = reshape(wrench.g,6*n,1);

% Reactional force factor (Largrange Multiplier)
lambda = zeros(5*nc + no + n_fix - nr, 1);
% lambda = zeros(5*nc + no,1);
wrench.react = zeros(6*n,1);
wrench.total = zeros(6*n,1);

%% Initial Conditions and Setup / Initialize book-keeping

s0 = [    BASE.x ;     BASE.q ;
        alpha1.x ;   alpha1.q ;   alpha2.x ;   alpha2.q ;
        gamma1.x ;   gamma1.q ;   gamma2.x ;   gamma2.q ;  
         beta1.x ;    beta1.q ;    beta2.x ;    beta2.q ; 
        delta1.x ;   delta1.q ;   delta2.x ;   delta2.q ; 
      epsilon1.x ; epsilon1.q ; epsilon2.x ; epsilon2.q];
% s0 = [BASE.x ; BASE.q ; alpha1.x ; alpha1.q ; alpha2.x ; alpha2.q ; gamma1.x ; gamma1.q ; gamma2.x ; gamma2.q ; beta1.x ; beta1.q ; beta2.x ; beta2.q ; delta1.x ; delta1.q ; delta2.x ; delta2.q];
% s0 = [BASE.x ; BASE.q ; alpha1.x ; alpha1.q ; alpha2.x ; alpha2.q ; gamma1.x ; gamma1.q ; gamma2.x ; gamma2.q];
% s0 = [BASE.x ; BASE.q ; alpha1.x ; alpha1.q ; alpha2.x ; alpha2.q];
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

% ----- Plot -----
% plot3(s(1,1),s(2,1),s(3,1),'ko','MarkerSize',5,'MarkerFaceColor','k'); hold on
% % plot3([B.x(1) points.A1_3d(1)],[B.x(2) points.A1_3d(2)],[B.x(3) points.A1_3d(3)],'r');
% plot3(s(8,1),s(9,1),s(10,1),'ro','MarkerSize',5,'MarkerFaceColor','r');
% % plot3(points.A1_3d(1),points.A1_3d(2),points.A1_3d(3),'ro','MarkerSize',5,'MarkerFaceColor','r');
% % plot3([B.x(1) points.A2_3d(1)],[B.x(2) points.A2_3d(2)],[B.x(3) points.A2_3d(3)],'b');
% plot3(s(15,1),s(16,1),s(17,1),'bo','MarkerSize',5,'MarkerFaceColor','b');
% % plot3(points.A2_3d(1),points.A2_3d(2),points.A2_3d(3),'bo','MarkerSize',5,'MarkerFaceColor','b');
% % plot3([points.E_3d(1) points.B1_3d(1)],[points.E_3d(2) points.B1_3d(2)],[points.E_3d(3) points.B1_3d(3)],'g');
% plot3(s(22,1),s(23,1),s(24,1),'go','MarkerSize',5,'MarkerFaceColor','g');
% plot3(s(29,1),s(30,1),s(31,1),'mo','MarkerSize',5,'MarkerFaceColor','m'); 
% arrow3d([s(8,1) s(8,1)+vector.alpha1_O1(1)],[s(9,1) s(9,1)+vector.alpha1_O1(2)],[s(10,1) s(10,1)+vector.alpha1_O1(3)],a,b,c,'r');
% arrow3d([s(15,1) s(15,1)+vector.alpha2_O2(1)],[s(16,1) s(16,1)+vector.alpha2_O2(2)],[s(17,1) s(17,1)+vector.alpha2_O2(3)],a,b,c,'b');
% plot3(s(36,1),s(37,1),s(38,1),'ro','MarkerSize',5,'MarkerFaceColor','r');
% plot3(s(43,1),s(44,1),s(45,1),'bo','MarkerSize',5,'MarkerFaceColor','b');
% plot3(points.A1_3d(1),points.A1_3d(2),points.A1_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.A2_3d(1),points.A2_3d(2),points.A2_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.B1_3d(1),points.B1_3d(2),points.B1_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.B2_3d(1),points.B2_3d(2),points.B2_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.C1_3d(1),points.C1_3d(2),points.C1_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.C2_3d(1),points.C2_3d(2),points.C2_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.D1_3d(1),points.D1_3d(2),points.D1_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.D2_3d(1),points.D2_3d(2),points.D2_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.F1_3d(1),points.F1_3d(2),points.F1_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.F2_3d(1),points.F2_3d(2),points.F2_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.E_3d(1),points.E_3d(2),points.E_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3(points.G_3d(1),points.G_3d(2),points.G_3d(3),'ko','MarkerSize',5,'MarkerFaceColor','k');
% plot3([points.E_3d(1) points.A1_3d(1)],[points.E_3d(2) points.A1_3d(2)],[points.E_3d(3) points.A1_3d(3)],'k');
% plot3([points.E_3d(1) points.A2_3d(1)],[points.E_3d(2) points.A2_3d(2)],[points.E_3d(3) points.A2_3d(3)],'k');
% plot3([B.x(1) points.B1_3d(1)],[B.x(2) points.B1_3d(2)],[B.x(3) points.B1_3d(3)],'k');
% plot3([B.x(1) points.B2_3d(1)],[B.x(2) points.B2_3d(2)],[B.x(3) points.B2_3d(3)],'k');
% plot3([points.C1_3d(1) points.D1_3d(1)],[points.C1_3d(2) points.D1_3d(2)],[points.C1_3d(3) points.D1_3d(3)],'k');
% plot3([points.C2_3d(1) points.D2_3d(1)],[points.C2_3d(2) points.D2_3d(2)],[points.C2_3d(3) points.D2_3d(3)],'k');
% 
% plot3(s(50,1),s(51,1),s(52,1),'co','MarkerSize',5,'MarkerFaceColor','c');
% plot3(s(57,1),s(58,1),s(59,1),'co','MarkerSize',5,'MarkerFaceColor','c');
% plot3(s(64,1),s(65,1),s(66,1),'ro','MarkerSize',5,'MarkerFaceColor','r');
% plot3(s(71,1),s(72,1),s(73,1),'bo','MarkerSize',5,'MarkerFaceColor','b');
% hold off
% axis equal
% axis([-150 150 -200 200 -350 100]);
% view(0,0);
% ----------------

% initialize book-keeping
history.s(:,1) = s;
history.v(:,1) = v;

for t = 1:time.step
    %% Rotate inertia tensor
    I_rot = zeros(3,3*n);
    for i = 0 : n-1
        I_rot(:, 3*i + 1 : 3*i + 3) = Rq(s(7*i+4:7*(i+1),1)) * I(:,3*i+1:3*(i+1)) * Rq(s(7*i+4:7*(i+1),1))';
        I_rot(:, 3*i + 1 : 3*i + 3) = I_rot(:, 3*i + 1 : 3*i + 3  ) - tril(I_rot(:, 3*i + 1 : 3*i + 3  ),-1) + triu(I_rot(:, 3*i + 1 : 3*i + 3  ),1)';
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

    %% Refresh wrenches
    for i = 0 : n-1
        w = v(6*i+4:6*(i+1),1);
        wrench.w(6*i+4:6*(i+1),1) = -skew(w) * I_rot(:,3*i+1 : 3*(i+1)) * w;
    end

    %% Construct the Jacobian
    % rotate ns_
    ns = zeros(3,3,n);
    for i = 0 : n-1
        ns(:,:,i+1) = Rq(s(7*i+4 : 7*i+7,1))*ns_(:,:,i+1);
    end

    % Jacobian matrices
    % O1 B(1)/alpha1(2)-5 constraints
    J_O1_B         = [  eye(3)             zeros(3,3)  ; zeros(1,3)  (skew(ns(:,1,1))*ns(:,2,2))'  ; zeros(1,3)  (skew(ns(:,3,1))*ns(:,2,2))'];
    J_O1_a1        = [ -eye(3)  skew(vector.alpha1_O1) ; zeros(1,3) -(skew(ns(:,1,1))*ns(:,2,2))'  ; zeros(1,3) -(skew(ns(:,3,1))*ns(:,2,2))'];
 
    % O2 B(1)/alpha2(3)-5 constraints
    J_O2_B         = [  eye(3)              zeros(3,3)   ; zeros(1,3)   (skew(ns(:,1,1))*ns(:,2,3))'  ; zeros(1,3)   (skew(ns(:,3,1))*ns(:,2,3))'];
    J_O2_a2        = [ -eye(3)  skew(vector.alpha2_O2)   ; zeros(1,3)  -(skew(ns(:,1,1))*ns(:,2,3))'  ; zeros(1,3)  -(skew(ns(:,3,1))*ns(:,2,3))'];

    % A1 alpha1(2)/gamma1(4)-5 constraints
    J_A1_a1        = [  eye(3)  -skew(vector.alpha1_A1)   ; zeros(1,3)   (skew(ns(:,1,2))*ns(:,2,4))'  ; zeros(1,3)  (skew(ns(:,3,2))*ns(:,2,4))'];
    J_A1_g1        = [ -eye(3)   skew(vector.gamma1_A1)   ; zeros(1,3)  -(skew(ns(:,1,2))*ns(:,2,4))'  ; zeros(1,3) -(skew(ns(:,3,2))*ns(:,2,4))'];
%     J_A1_a1        = [  eye(3)  -skew(vector.alpha1_A1)   ; zeros(1,3)   (skew(ns(:,1,2))*ns(:,2,4))'];
%     J_A1_g1        = [ -eye(3)   skew(vector.gamma1_A1)   ; zeros(1,3)  -(skew(ns(:,1,2))*ns(:,2,4))'];

    % A2 alpha2(3)/gamma2(5)-5 constraints
    J_A2_a2        = [  eye(3)  -skew(vector.alpha2_A2)   ; zeros(1,3)   (skew(ns(:,1,3))*ns(:,2,5))'  ; zeros(1,3)   (skew(ns(:,3,3))*ns(:,2,5))'];
    J_A2_g2        = [ -eye(3)   skew(vector.gamma2_A2)   ; zeros(1,3)  -(skew(ns(:,1,3))*ns(:,2,5))'  ; zeros(1,3)  -(skew(ns(:,3,3))*ns(:,2,5))'];
%     J_A2_a2        = [  eye(3)  -skew(vector.alpha2_A2)   ; zeros(1,3)   (skew(ns(:,1,3))*ns(:,2,5))'];  % xy
%     J_A2_g2        = [ -eye(3)   skew(vector.gamma2_A2)   ; zeros(1,3)  -(skew(ns(:,1,3))*ns(:,2,5))'];  % xy

    % E gamma1(4)/gamma2(5)-2 constraints(x z)
%     J_E_g1         = [  eye(3)  -skew(vector.gamma1_E)    ; zeros(1,3)   (skew(ns(:,1,4))*ns(:,2,5))'  ; zeros(1,3)   (skew(ns(:,3,4))*ns(:,2,5))'];
%     J_E_g2         = [ -eye(3)   skew(vector.gamma2_E)    ; zeros(1,3)  -(skew(ns(:,1,4))*ns(:,2,5))'  ; zeros(1,3)  -(skew(ns(:,3,4))*ns(:,2,5))'];
%     J_E_g1         = [  eye(3)  -skew(vector.gamma1_E)  ; zeros(1,3)   (skew(ns(:,3,4))*ns(:,2,5))'];  % xz
%     J_E_g2         = [ -eye(3)   skew(vector.gamma2_E)  ; zeros(1,3)  -(skew(ns(:,3,4))*ns(:,2,5))'];  % xz
%     J_E_g1         = [  eye(3)  -skew(vector.gamma1_E)    ; zeros(1,3)   (skew(ns(:,1,4))*ns(:,2,5))']; % xy
%     J_E_g2         = [ -eye(3)   skew(vector.gamma2_E)    ; zeros(1,3)  -(skew(ns(:,1,4))*ns(:,2,5))']; % xy
    jz = [1  0  0 ; 0  0  1];
    skew_vector.gamma1_E = skew(vector.gamma1_E);
    skew_vector.gamma2_E = skew(vector.gamma2_E);
    J_E_g1         = [ jz  [-skew_vector.gamma1_E(1,:) ; -skew_vector.gamma1_E(3,:)]];
    J_E_g2         = [-jz  [ skew_vector.gamma2_E(1,:) ;  skew_vector.gamma2_E(3,:)]];

    % B1 alpha1(2)/beta1(6)-5 constraints
    J_B1_a1        = [  eye(3)  -skew(vector.alpha1_B1)   ; zeros(1,3)   (skew(ns(:,1,2))*ns(:,2,6))'  ; zeros(1,3)   (skew(ns(:,3,2))*ns(:,2,6))'];
    J_B1_b1        = [ -eye(3)   skew(vector.beta1_B1)    ; zeros(1,3)  -(skew(ns(:,1,2))*ns(:,2,6))'  ; zeros(1,3)  -(skew(ns(:,3,2))*ns(:,2,6))'];
%     J_B1_a1        = [  eye(3)  -skew(vector.alpha1_B1)   ; zeros(1,3)   (skew(ns(:,1,2))*ns(:,2,6))']; % xy
%     J_B1_b1        = [ -eye(3)   skew(vector.beta1_B1)    ; zeros(1,3)  -(skew(ns(:,1,2))*ns(:,2,6))']; % xy

    % B2 alpha2(3)/beta2(7)-5 constraints
    J_B2_a2        = [  eye(3)  -skew(vector.alpha2_B2)   ; zeros(1,3)   (skew(ns(:,1,3))*ns(:,2,7))'  ; zeros(1,3)   (skew(ns(:,3,3))*ns(:,2,7))'];
    J_B2_b2        = [ -eye(3)   skew(vector.beta2_B2)    ; zeros(1,3)  -(skew(ns(:,1,3))*ns(:,2,7))'  ; zeros(1,3)  -(skew(ns(:,3,3))*ns(:,2,7))'];
%     J_B2_a2        = [  eye(3)  -skew(vector.alpha2_B2)   ; zeros(1,3)   (skew(ns(:,1,3))*ns(:,2,7))']; % xy
%     J_B2_b2        = [ -eye(3)   skew(vector.beta2_B2)    ; zeros(1,3)  -(skew(ns(:,1,3))*ns(:,2,7))']; % xy

    % D1 gamma1(4)/delta1(8)-5 constraints
    J_D1_g1        = [  eye(3)  -skew(vector.gamma1_D1)   ; zeros(1,3)   (skew(ns(:,1,4))*ns(:,2,8))'  ; zeros(1,3)   (skew(ns(:,3,4))*ns(:,2,8))'];
    J_D1_d1        = [ -eye(3)   skew(vector.delta1_D1)   ; zeros(1,3)  -(skew(ns(:,1,4))*ns(:,2,8))'  ; zeros(1,3)  -(skew(ns(:,3,4))*ns(:,2,8))'];
%     J_D1_g1        = [  eye(3)  -skew(vector.gamma1_D1)   ; zeros(1,3)   (skew(ns(:,1,4))*ns(:,2,8))']; % xy
%     J_D1_d1        = [ -eye(3)   skew(vector.delta1_D1)   ; zeros(1,3)  -(skew(ns(:,1,4))*ns(:,2,8))']; % xy

    % D2 gamma2(5)/delta2(9)-5 constraints
    J_D2_g2        = [  eye(3)  -skew(vector.gamma2_D2)   ; zeros(1,3)   (skew(ns(:,1,5))*ns(:,2,9))'  ; zeros(1,3)   (skew(ns(:,3,5))*ns(:,2,9))'];
    J_D2_d2        = [ -eye(3)   skew(vector.delta2_D2)   ; zeros(1,3)  -(skew(ns(:,1,5))*ns(:,2,9))'  ; zeros(1,3)  -(skew(ns(:,3,5))*ns(:,2,9))'];
%     J_D2_g2        = [  eye(3)  -skew(vector.gamma2_D2)   ; zeros(1,3)   (skew(ns(:,1,5))*ns(:,2,9))']; % xy
%     J_D2_d2        = [ -eye(3)   skew(vector.delta2_D2)   ; zeros(1,3)  -(skew(ns(:,1,5))*ns(:,2,9))']; % xy

    % C1 beta1(6)/delta1(8)-2 constraints
%     J_C1_b1        = [  eye(3)  -skew(vector.beta1_C1)    ; zeros(1,3)   (skew(ns(:,1,6))*ns(:,2,8))'  ; zeros(1,3)   (skew(ns(:,3,6))*ns(:,2,8))'];
%     J_C1_d1        = [ -eye(3)   skew(vector.delta1_C1)   ; zeros(1,3)  -(skew(ns(:,1,6))*ns(:,2,8))'  ; zeros(1,3)  -(skew(ns(:,3,6))*ns(:,2,8))'];
%     J_C1_b1        = [  eye(3)  -skew(vector.beta1_C1)    ; zeros(1,3)   (skew(ns(:,1,6))*ns(:,2,8))']; % xy
%     J_C1_d1        = [ -eye(3)   skew(vector.delta1_C1)   ; zeros(1,3)  -(skew(ns(:,1,6))*ns(:,2,8))']; % xy
    skew_vector.beta1_C1 = skew(vector.beta1_C1);
    skew_vector.delta1_C1 = skew(vector.delta1_C1);
    J_C1_b1        = [ jz  [-skew_vector.beta1_C1(1,:)  ; -skew_vector.beta1_C1(3,:) ] ];
    J_C1_d1        = [-jz  [ skew_vector.delta1_C1(1,:) ;  skew_vector.delta1_C1(3,:)] ];    

    % C2 beta2(7)/delta2(9)-2 constraints
%     J_C2_b2        = [  eye(3)  -skew(vector.beta2_C2)    ; zeros(1,3)   (skew(ns(:,1,7))*ns(:,2,9))'  ; zeros(1,3)   (skew(ns(:,3,7))*ns(:,2,9))'];
%     J_C2_d2        = [ -eye(3)   skew(vector.delta2_C2)   ; zeros(1,3)  -(skew(ns(:,1,7))*ns(:,2,9))'  ; zeros(1,3)  -(skew(ns(:,3,7))*ns(:,2,9))'];
%     J_C2_b2        = [  eye(3)  -skew(vector.beta2_C2)    ; zeros(1,3)   (skew(ns(:,1,7))*ns(:,2,9))']; % xy
%     J_C2_d2        = [ -eye(3)   skew(vector.delta2_C2)   ; zeros(1,3)  -(skew(ns(:,1,7))*ns(:,2,9))']; % xy
    skew_vector.beta2_C2 = skew(vector.beta2_C2);
    skew_vector.delta2_C2 = skew(vector.delta2_C2);
    J_C2_b2        = [ jz  [-skew_vector.beta2_C2(1,:)  ; -skew_vector.beta2_C2(3,:) ] ];
    J_C2_d2        = [-jz  [ skew_vector.delta2_C2(1,:) ;  skew_vector.delta2_C2(3,:)] ];    

    % F1 beta1(6)/epsilon1(10)-5 constraints
    J_F1_b1        = [  eye(3)  -skew(vector.beta1_F1)    ; zeros(1,3)   (skew(ns(:,1,6))*ns(:,2,10))' ; zeros(1,3)    (skew(ns(:,3,6))*ns(:,2,10))'];
    J_F1_e1        = [ -eye(3)   skew(vector.epsilon1_F1) ; zeros(1,3)  -(skew(ns(:,1,6))*ns(:,2,10))' ; zeros(1,3)   -(skew(ns(:,3,6))*ns(:,2,10))'];
%     J_F1_b1        = [  eye(3)  -skew(vector.beta1_F1)    ; zeros(1,3)   (skew(ns(:,1,6))*ns(:,2,10))']; % xy
%     J_F1_e1        = [ -eye(3)   skew(vector.epsilon1_F1) ; zeros(1,3)  -(skew(ns(:,1,6))*ns(:,2,10))']; % xy

    % F2 beta2(7)/epsilon2(11)-5 constraints
    J_F2_b2        = [  eye(3)  -skew(vector.beta2_F2)    ; zeros(1,3)   (skew(ns(:,1,7))*ns(:,2,11))' ; zeros(1,3)    (skew(ns(:,3,7))*ns(:,2,11))'];
    J_F2_e2        = [ -eye(3)   skew(vector.epsilon2_F2) ; zeros(1,3)  -(skew(ns(:,1,7))*ns(:,2,11))' ; zeros(1,3)   -(skew(ns(:,3,7))*ns(:,2,11))'];
%     J_F2_b2        = [  eye(3)  -skew(vector.beta2_F2)    ; zeros(1,3)   (skew(ns(:,1,7))*ns(:,2,11))']; % xy
%     J_F2_e2        = [ -eye(3)   skew(vector.epsilon2_F2) ; zeros(1,3)  -(skew(ns(:,1,7))*ns(:,2,11))']; % xy

    % G epsilon1(10)/epsilon2(11)-2 constraints
%     J_G_e1         = [  eye(3)  -skew(vector.epsilon1_G)  ; zeros(1,3)   (skew(ns(:,1,10))*ns(:,2,11))' ; zeros(1,3)   (skew(ns(:,3,10))*ns(:,2,11))'];
%     J_G_e2         = [ -eye(3)   skew(vector.epsilon2_G)  ; zeros(1,3)  -(skew(ns(:,1,10))*ns(:,2,11))' ; zeros(1,3)  -(skew(ns(:,3,10))*ns(:,2,11))'];
%     J_G_e1         = [  eye(3)  -skew(vector.epsilon1_G)  ; zeros(1,3)   (skew(ns(:,1,10))*ns(:,2,11))']; % xy
%     J_G_e2         = [ -eye(3)   skew(vector.epsilon2_G)  ; zeros(1,3)  -(skew(ns(:,1,10))*ns(:,2,11))']; % xy
    skew_vector.epsilon1_G = skew(vector.epsilon1_G);
    skew_vector.epsilon2_G = skew(vector.epsilon2_G);
    J_G_e1         = [ jz  [-skew_vector.epsilon1_G(1,:) ; -skew_vector.epsilon1_G(3,:)]];
    J_G_e2         = [-jz  [ skew_vector.epsilon2_G(1,:) ;  skew_vector.epsilon2_G(3,:)]];

    % Restrict Rotation
    % O1 B/alpha1
    Jr_O1_B          = [zeros(1,3)   ns(:,2,1)'];
    Jr_O1_a1         = [zeros(1,3)  -ns(:,2,2)'];

    % O2 B/alpha2
    Jr_O2_B          = [zeros(1,3)   ns(:,2,1)'];
    Jr_O2_a2         = [zeros(1,3)  -ns(:,2,3)'];

    % fix BASE(B) to O
    J_BASE_O = [eye(3) -skew(BASE.BASE_O) zeros(3,(n-1)*6) ; zeros(3,3) eye(3) zeros(3,(n-1)*6)];

    X = zeros(5, 6);
    x = zeros(2, 6);
    z = zeros(1, 6);

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2, epsilon1,2(fixed) remove redundancies
%     J = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X         X        X;
%          J_O2_B         X   J_O2_a2        X        X         X        X         X        X         X        X;
%         Jr_O1_B  Jr_O1_a1         z        z        z         z        z         z        z         z        z;
%         Jr_O2_B         z  Jr_O2_a2        z        z         z        z         z        z         z        z;
%               x   J_A1_a1         x  J_A1_g1        x         x        x         x        x         x        x;
%               x         x   J_A2_a2        x  J_A2_g2         x        x         x        x         x        x;
%               x         x         x   J_E_g1   J_E_g2         x        x         x        x         x        x;
%               x   J_B1_a1         x        x        x   J_B1_b1        x         x        x         x        x;
%               x         x   J_B2_a2        x        x         x  J_B2_b2         x        x         x        x;
%               x         x         x        x        x   J_C1_b1        x   J_C1_d1        x         x        x;
%               x         x         x        x        x         x  J_C2_b2         x  J_C2_d2         x        x;
%               x         x         x  J_D1_g1        x         x        x   J_D1_d1        x         x        x;
%               x         x         x        x  J_D2_g2         x        x         x  J_D2_d2         x        x;
%               x         x         x        x        x   J_F1_b1        x         x        x   J_F1_e1        x;
%               x         x         x        x        x         x  J_F2_b2         x        x         x  J_F2_e2;
%               x         x         x        x        x         x        x         x        x    J_G_e1   J_G_e2;
%        J_BASE_O]; 

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2, epsilon1,2(fixed)
%     J = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X         X        X;
%          J_O2_B         X   J_O2_a2        X        X         X        X         X        X         X        X;
%         Jr_O1_B  Jr_O1_a1         x        x        x         x        x         x        x         x        x;
%         Jr_O2_B         x  Jr_O2_a2        x        x         x        x         x        x         x        x;
%               X   J_A1_a1         X  J_A1_g1        X         X        X         X        X         X        X;
%               X         X   J_A2_a2        X  J_A2_g2         X        X         X        X         X        X;
%               X         X         X   J_E_g1   J_E_g2         X        X         X        X         X        X;
%               X   J_B1_a1         X        X        X   J_B1_b1        X         X        X         X        X;
%               X         X   J_B2_a2        X        X         X  J_B2_b2         X        X         X        X;
%               X         X         X        X        X   J_C1_b1        X   J_C1_d1        X         X        X;
%               X         X         X        X        X         X  J_C2_b2         X  J_C2_d2         X        X;
%               X         X         X  J_D1_g1        X         X        X   J_D1_d1        X         X        X;
%               X         X         X        X  J_D2_g2         X        X         X  J_D2_d2         X        X;
%               X         X         X        X        X   J_F1_b1        X         X        X   J_F1_e1        X;
%               X         X         X        X        X         X  J_F2_b2         X        X         X  J_F2_e2;
%               X         X         X        X        X         X        X         X        X    J_G_e1   J_G_e2;
%        J_BASE_O];       

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2, epsilon1,2(freely) remove redundancies
    J = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X         X        X;
         J_O2_B         X   J_O2_a2        X        X         X        X         X        X         X        X;
              X   J_A1_a1         X  J_A1_g1        X         X        X         X        X         X        X;
              X         X   J_A2_a2        X  J_A2_g2         X        X         X        X         X        X;
              x         x         x   J_E_g1   J_E_g2         x        x         x        x         x        x;
              X   J_B1_a1         X        X        X   J_B1_b1        X         X        X         X        X;
              X         X   J_B2_a2        X        X         X  J_B2_b2         X        X         X        X;
              x         x         x        x        x   J_C1_b1        x   J_C1_d1        x         x        x;
              x         x         x        x        x         x  J_C2_b2         x  J_C2_d2         x        x;
              X         X         X  J_D1_g1        X         X        X   J_D1_d1        X         X        X;
              X         X         X        X  J_D2_g2         X        X         X  J_D2_d2         X        X;
              X         X         X        X        X   J_F1_b1        X         X        X   J_F1_e1        X;
              X         X         X        X        X         X  J_F2_b2         X        X         X  J_F2_e2;
              x         x         x        x        x         x        x         x        x    J_G_e1   J_G_e2;
       J_BASE_O]; 

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2, epsilon1,2(freely)
%     J = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X         X        X;
%          J_O2_B         X   J_O2_a2        X        X         X        X         X        X         X        X;
%               X   J_A1_a1         X  J_A1_g1        X         X        X         X        X         X        X;
%               X         X   J_A2_a2        X  J_A2_g2         X        X         X        X         X        X;
%               X         X         X   J_E_g1   J_E_g2         X        X         X        X         X        X;
%               X   J_B1_a1         X        X        X   J_B1_b1        X         X        X         X        X;
%               X         X   J_B2_a2        X        X         X  J_B2_b2         X        X         X        X;
%               X         X         X        X        X   J_C1_b1        X   J_C1_d1        X         X        X;
%               X         X         X        X        X         X  J_C2_b2         X  J_C2_d2         X        X;
%               X         X         X  J_D1_g1        X         X        X   J_D1_d1        X         X        X;
%               X         X         X        X  J_D2_g2         X        X         X  J_D2_d2         X        X;
%               X         X         X        X        X   J_F1_b1        X         X        X   J_F1_e1        X;
%               X         X         X        X        X         X  J_F2_b2         X        X         X  J_F2_e2;
%               X         X         X        X        X         X        X         X        X    J_G_e1   J_G_e2;
%        J_BASE_O]; 

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2(fixed)
%     J = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X;
%          J_O2_B         X   J_O2_a2        X        X         X        X         X        X;
%         Jr_O1_B  Jr_O1_a1         x        x        x         x        x         x        x;
%         Jr_O2_B         x  Jr_O2_a2        x        x         x        x         x        x;
%               X   J_A1_a1         X  J_A1_g1        X         X        X         X        X;
%               X         X   J_A2_a2        X  J_A2_g2         X        X         X        X;
%               X         X         X   J_E_g1   J_E_g2         X        X         X        X;
%               X   J_B1_a1         X        X        X   J_B1_b1        X         X        X;
%               X         X   J_B2_a2        X        X         X  J_B2_b2         X        X;
%               X         X         X        X        X   J_C1_b1        X   J_C1_d1        X;
%               X         X         X        X        X         X  J_C2_b2         X  J_C2_d2;
%               X         X         X  J_D1_g1        X         X        X   J_D1_d1        X;
%               X         X         X        X  J_D2_g2         X        X         X  J_D2_d2;
%        J_BASE_O];    

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2(freely) remove redundancies
%     J = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X;
%          J_O2_B         X   J_O2_a2        X        X         X        X         X        X;
%               x   J_A1_a1         x  J_A1_g1        x         x        x         x        x;
%               x         x   J_A2_a2        x  J_A2_g2         x        x         x        x;
%               x         x         x   J_E_g1   J_E_g2         x        x         x        x;
%               x   J_B1_a1         x        x        x   J_B1_b1        x         x        x;
%               x         x   J_B2_a2        x        x         x  J_B2_b2         x        x;
%               x         x         x        x        x   J_C1_b1        x   J_C1_d1        x;
%               x         x         x        x        x         x  J_C2_b2         x  J_C2_d2;
%               x         x         x  J_D1_g1        x         x        x   J_D1_d1        x;
%               x         x         x        x  J_D2_g2         x        x         x  J_D2_d2;
%        J_BASE_O];

    % Jacobian of Body, alpha1,2, gamma1,2, beta1,2, delta1,2(freely)
%     J = [J_O1_B   J_O1_a1         X        X        X         X        X         X        X;
%          J_O2_B         X   J_O2_a2        X        X         X        X         X        X;
%               X   J_A1_a1         X  J_A1_g1        X         X        X         X        X;
%               X         X   J_A2_a2        X  J_A2_g2         X        X         X        X;
%               X         X         X   J_E_g1   J_E_g2         X        X         X        X;
%               X   J_B1_a1         X        X        X   J_B1_b1        X         X        X;
%               X         X   J_B2_a2        X        X         X  J_B2_b2         X        X;
%               X         X         X        X        X   J_C1_b1        X   J_C1_d1        X;
%               X         X         X        X        X         X  J_C2_b2         X  J_C2_d2;
%               X         X         X  J_D1_g1        X         X        X   J_D1_d1        X;
%               X         X         X        X  J_D2_g2         X        X         X  J_D2_d2;
%        J_BASE_O];  

    % Jacobian of Body, alpha1,2, gamma1,2(fixed)
%     J = [J_O1_B   J_O1_a1         X        X        X;
%          J_O2_B         X   J_O2_a2        X        X;
%         Jr_O1_B  Jr_O1_a1         x        x        x;
%         Jr_O2_B         x  Jr_O2_a2        x        x;
%               X   J_A1_a1         X  J_A1_g1        X;
%               X         X   J_A2_a2        X  J_A2_g2;
%               X         X         X   J_E_g1   J_E_g2;
%        J_BASE_O];

    % Jacobian of Body, alpha1,2, gamma1,2(freely)
%     J = [J_O1_B   J_O1_a1         X        X        X;
%          J_O2_B         X   J_O2_a2        X        X;
%               x   J_A1_a1         x  J_A1_g1        x;
%               x         x   J_A2_a2        x  J_A2_g2;
%               x         x         x   J_E_g1   J_E_g2;
%        J_BASE_O];    

    % Jacobian of Body, alpha1,2(fixed)
%     J = [ J_O1_B    J_O1_a1         X;
%           J_O2_B          X   J_O2_a2;
%          Jr_O1_B   Jr_O1_a1         x;
%          Jr_O2_B          x  Jr_O2_a2;
%          J_BASE_O];

    % Jacobian of Body, alpha1,2(freely)
%     J = [ J_O1_B    J_O1_a1         X;
%           J_O2_B          X   J_O2_a2;
%          J_BASE_O];    

    %% Calculate Reactional force
    v_ = v + time.period * W * (wrench.ext + wrench.g + wrench.w);
    K = J * W * J';
    K = K - tril(K,-1) + triu(K,1)'; % Make sure K is still symmetric
%     lambda(:,1) = -inv(K)*(J*v_) / time.period;
    lambda(:,1) = -K\(J*v_) / time.period;
    wrench.react = J'*lambda;
    wrench.total = wrench.ext + wrench.g + wrench.w + wrench.react;

    %% Refresh system states
    v = v + time.period * W * wrench.total;
    % s = s + time.period * S * v;
    
    for i = 0 : n-1
        % Update COM position
        s(i*7+1 : i*7+3) = s(i*7+1 : i*7+3) + time.period * v(i*6+1 : i*6+3);

        % Update quaternion (quaternion multiplication make sure quaternion do not exceed 1)
        [axis_,rot_] = Vec_to_Axis_Deg(v(i*6+4 : i*6+6));
        s(i*7+4 : i*7+7) = skew([cos(time.period*rot_/2);sin(time.period*rot_/2)*axis_])*s(i*7+4 : i*7+7);
    end
    % s(1:3) = s(1:3) + time.period * v(1:3);
    % s(8:10) = s(8:10) + time.period * v(8:10);
    % s(15:18) = s(15:18) + time.period * v(14:16);
    
    % [axis1,rot1] = Vec_to_Axis_Deg(v(4:6));
    % s(4:7) = skew([cos(time.period*rot1/2);sin(time.period*rot1/2)*axis1])*s(4:7);
    % [axis2,rot2] = Vec_to_Axis_Deg(v(10:12));
    % s(11:14) = skew([cos(time.period*rot2/2);sin(time.period*rot2/2)*axis2])*s(11:14);

    %% Analyze joint torque
    Fr_O1_B = Jr_O1_B'*lambda(11,1);
    Fr_O1_a1 = Jr_O1_a1'*lambda(11,1);

    Fr_O2_B = Jr_O2_B'*lambda(12,1);
    Fr_O2_a2 = Jr_O2_a2'*lambda(12,1);

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

    % ---------- Plot ----------
    plot3(s(1,1),s(2,1),s(3,1),'Marker','o','MarkerSize',5,'MarkerFaceColor','k','MarkerEdgeColor','none'); hold on
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

    [rupframe_3D,rdownframe_3D] = outframe_coordinate3D(s(36:38,1)+vector.beta1_B1, s(36:38,1)+vector.beta1_C1, s(36:38,1)+vector.beta1_F1, s(64:66,1)+vector.epsilon1_G,R,up_deg);
    plot3(rupframe_3D(:,1),rupframe_3D(:,2),rupframe_3D(:,3),'LineWidth',2,'Color','#0000C6');
    plot3(rdownframe_3D(:,1),rdownframe_3D(:,2),rdownframe_3D(:,3),'LineWidth',2,'Color','#5B00AE');

    [lupframe_3D,ldownframe_3D] = outframe_coordinate3D(s(43:45,1)+vector.beta2_B2, s(43:45,1)+vector.beta2_C2, s(43:45,1)+vector.beta2_F2, s(71:73,1)+vector.epsilon2_G,R,up_deg);
    plot3(lupframe_3D(:,1),lupframe_3D(:,2),lupframe_3D(:,3),'LineWidth',2,'Color','#F75000');
    plot3(ldownframe_3D(:,1),ldownframe_3D(:,2),ldownframe_3D(:,3),'LineWidth',2,'Color','#007500');

    hold off
    axis equal
    axis([-150 150 -200 200 -350 100]);
    view(0,0);
    drawnow;
    % --------------------------

    % ----- save to gif -----
%     frame = getframe(1);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     filename = '3bar_freely_rotate.gif';
%     if t == 1
%         imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',del);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',del);
%     end

    % ----- save to video(avi) -----
%     frame = getframe(gcf);
%     writeVideo(writerObj, frame);

    %% Update book-keeping list
    history.s(:,t+1) = s;
    history.v(:,t+1) = v;
    history.J(:,:,t+1) = J;
    history.lambda(:,t+1) = lambda;
    history.wrench.ext(:,t+1) = wrench.ext;
    history.wrench.w(:,t+1) = wrench.w;
    history.wrench.g(:,t+1) = wrench.g;
    history.wrench.react(:,t+1) = wrench.react;
    history.wrench.total(:,t+1) = wrench.total;

end