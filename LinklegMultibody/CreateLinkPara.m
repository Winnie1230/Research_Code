clear all; close all; clc;
% create Link parameters
%%
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

%% Linkage configurations (Using metic units - mmgs)
% Assume BASE is fixed to O(fix joint)

% ---------- Mass BASE(B) ----------
BASE.h = 100; % Height (Assume B is a solid cuboid)
BASE.w = 100; % Width
BASE.d = 100; % Depth
BASE.m = 3200; % Mass
BASE.I = BASE.m/12 * [ BASE.h^2+BASE.d^2 0 0 ; 0 BASE.w^2+BASE.d^2 0 ; 0 0 BASE.w^2+BASE.h^2 ]; % Inertia Tensor
BASE.W = [(1/BASE.m)*eye(3) zeros(3,3);zeros(3,3) inv(BASE.I)]; % Inverse inertia matrix

BASE.B_O = [0 0 0]';

% sliding joint
BASE.anchor = [0 0 -100]';


% ---------- Linkage alpha-1(Rigid Body) ----------
alpha1.m = 49.438724 ;
alpha1.I = [5226.992514	 5575.651755	 30.511514 ;
            5575.651755	 52731.315884	 2.550826 ;
            30.511514	 2.550826	     51270.275795 ];

% alpha1.I = [ 5226.992514	 5575.651755	 -30.511514 ;
% 	         5575.651755	 52731.315884	   2.550826 ;
% 	          -30.511514	 2.550826	   51270.275795 ];

alpha1.W = [(1/alpha1.m)*eye(3) zeros(3,3);zeros(3,3) inv(alpha1.I)]; % Inverse inertia matrix
alpha1.alpha1_O1 = [-36.45 0 0.02]'; % [-36.45 -8.63 0.02]'; % The vector pointing from the C.O.M of alpha-1 to O in the local frame.
% alpha1.alpha1_O1 = [-36.45 -8.63 0.02]'; % local frame



% ---------- Linkage alpha-2(Rigid Body) -----------
% alpha2.m = 49.438724 ;  %48.332901;
alpha2.m = 48.332901;
alpha2.I = [5226.992514	 5575.651755	 30.511514 ;
            5575.651755	 52731.315884	 2.550826 ;
            30.511514	 2.550826	     51270.275795 ]; 
%             [  5320.371779	5878.366912      -26.243856;
%         	 -5878.366912	53989.169452	   1.415240;
% 	           -26.243856	    1.415240   52405.042766];

% alpha2.I = [  5320.372467   -5878.155734      26.348805;
%              -5878.155734   53989.571406       1.355846;
%                 26.348805       1.355846   52405.385266];
alpha2.W = [(1/alpha2.m)*eye(3) zeros(3,3);zeros(3,3) inv(alpha2.I)];
alpha2.alpha2_O2 = [-36.43   0   -0.02 ]';
% alpha2.alpha2_O2 = [-36.45 -8.63 -0.02]'; % local frame


% ---------- Linkage gamma-1(Rigid Body) ----------
gamma1.m =  49.049581 ;
% gamma1.m =  0.001 ;
gamma1.I =[ 2956.157949	    0.004740	  1300.105736 ;
            0.004740	    92320.757630  -0.000375 ;
            1300.105736	    -0.000375	  90182.095841];
 
gamma1.W = [(1/gamma1.m)*eye(3) zeros(3,3);zeros(3,3) inv(gamma1.I)];
gamma1.gamma1_A1 = [-65.68 0 -4.57]'; %[-65.68 5 -4.57]';
% gamma1.gamma1_A1 = [-65.68 5 -4.57]';  % local frame


% ---------- Linkage gamma-2(Rigid Body) ----------
gamma2.m = 49.049581;
% gamma2.m = 0.001;
gamma2.I = [ 2956.157949	  -0.004740	   -1300.105736 ;
            -0.004740	     92320.757630	  -0.000375 ;
            -1300.105736	   -0.000375	  90182.095841];

gamma2.W = [(1/gamma2.m)*eye(3) zeros(3,3);zeros(3,3) inv(gamma2.I)];
gamma2.gamma2_A2 =  [-65.68 0 4.57]'; 
% gamma2.gamma2_A2 =  [-65.68 -5 4.57]'; %local frame


% ----------- Linkage beta-1(Rigid Body) ----------
beta1.m = 142.168440 + 50;
beta1.I = [ 91843.636529	 -772.772852	 40829.647339 ;
            -772.772852    517848.015598	 -201.475342  ;
    	    40829.647339	 -201.475342	 493681.548715];

beta1.W = [(1/beta1.m)*eye(3) zeros(3,3);zeros(3,3) inv(beta1.I)];
beta1.beta1_B1 = [-75.5 0 -28.66]'; % local frame


% ----------- Linkage beta-2(Rigid Body) ----------
beta2.m = 142.168440 + 50;
beta2.I = [ 91843.636529	  772.772852	 -40829.647339 ;
             772.772852    517848.015598	 -201.475342  ;
    	    -40829.647339	 -201.475342	 493681.548715];

beta2.W = [(1/beta2.m)*eye(3) zeros(3,3);zeros(3,3) inv(beta2.I)];
beta2.beta2_B2 = [-75.5 0 28.66]'; % local frame


% ---------- Linkage delta-1(Rigid Body) ----------
delta1.m =  15.681471 ;
delta1.I = [  352.062649	 0.000000	    0.000000;
                0.000000	 13277.939609	 0.000000;
                0.000000	 0.000000	 13187.234818];
 
delta1.W = [(1/delta1.m)*eye(3) zeros(3,3);zeros(3,3) inv(delta1.I)];
delta1.delta1_D1 =  [-42.6 0 0]'; % [-42.6 5 0]';


% ---------- Linkage delta-2(Rigid Body) ----------
delta2.m =   15.681471;
delta2.I = [  352.062649	 0.000000	    0.000000;
                0.000000	 13277.939609	 0.000000;
                0.000000	 0.000000	 13187.234818];

delta2.W = [(1/delta2.m)*eye(3) zeros(3,3);zeros(3,3) inv(delta2.I)];
delta2.delta2_D2 =  [-42.6 0 0]'; % [-42.6 -5 0]';


% ---------- Linkage epsilon-1(Rigid Body) ----------
epsilon1.m =  66.713674 + 50;
epsilon1.I = [10744.530879   1691.358784	   735.827120 ;
               1691.358784	  57872.558479	  779.660039 ;
                735.827120	  779.660039	  58548.216359];

epsilon1.W = [(1/epsilon1.m)*eye(3) zeros(3,3);zeros(3,3) inv(epsilon1.I)];
epsilon1.epsilon1_F1 = [-44.1 0 -7.6]';


% ---------- Linkage epsilon-2(Rigid Body) ----------
epsilon2.m =  66.713674 + 50;
epsilon2.I = [10744.530879    -1691.358784	   -735.827120 ;
              -1691.358784	  57872.558479	    779.660039 ;
               -735.827120	    779.660039	  58548.216359];

epsilon2.W = [(1/epsilon2.m)*eye(3) zeros(3,3);zeros(3,3) inv(epsilon2.I)];
epsilon2.epsilon2_F2 = [-44.1 0 7.6]';

filename = 'LinkPara_R100.mat';
save(filename,"alpha1","alpha2","beta1","beta2","delta1","delta2","epsilon1","epsilon2","gamma1","gamma2","BASE");



