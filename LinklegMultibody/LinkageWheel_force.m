%% Calculating all the force/torque apples on each components of the linkage -wheel leg by constrained dynamics

%% General Parameters (Using metic units - mmgs)

% Time
time.period = 0.001;

% Useful Constants
n = 11; % Number of linkages

%Environment
g = 9810 ;      % Gravitaional acceleration

%% Linkage configurations (Using metic units - mmgs)

% Mass O 

O.h = 100; % Height (Assume O is a solid cuboid)
O.w = 100; % Width
O.d = 100; % Depth
O.x = [0 0 0]'; % Position of mass O
O.theta = 0 ;
O.q = [cos(O.theta/4) 0 0 sin(O.theta/4)]'; % Orientation of mass O (quaternion)
O.m = 10000; % Mass
O.I = O.m/12 * [ O.h^2+O.d^2 0 0 ; 0 O.w^2+O.d^2 0 ; 0 0 O.w^2+O.h^2 ]; % Inertia Tensor
O.W = [(1/O.m)*eye(3) zeros(3,3);zeros(3,3) inv(O.I)]; % Inverse inertia matrix

% Linkage alpha-1

alpha1.m = 49.438724 ;
alpha1.I = [5226.992514	 5575.651755	 30.511514 ;
	        5575.651755	 52731.315884	 2.550826 ;
	        30.511514	 2.550826	     51270.275795 ];

alpha1.alpha1_O  = [-36.45 0 0.02]'; % [-36.45 -8.63 0.02]'; % The vector pointing from the C.O.M of alpha-1 to O in the local frame.
alpha1.nx = [1 0 0]'; % Unit vectors in local frame that form the constraints of rotational joints.
alpha1.ny = [0 1 0]';
alpha1.nz = [0 0 1]';
alpha1.W = [(1/alpha1.m)*eye(3) zeros(3,3);zeros(3,3) inv(alpha1.I)]; % Inverse inertia matrix

% Linkage alpha-2

alpha2.m = 49.438724 ;  %48.332901;
alpha2.I = [5226.992514	 5575.651755	 30.511514 ;
	        5575.651755	 52731.315884	 2.550826 ;
	        30.511514	 2.550826	     51270.275795 ]; 
%             [  5320.371779	5878.366912      -26.243856;
%         	 -5878.366912	53989.169452	   1.415240;
% 	           -26.243856	    1.415240   52405.042766];
alpha2.alpha2_O  = [-36.43   0   -0.02 ]'; %[-36.43   8.69   -0.02 ]'; 
alpha2.nx = [1 0 0]';
alpha2.ny = [0 1 0]';
alpha2.nz = [0 0 1]';
alpha2.W = [(1/alpha2.m)*eye(3) zeros(3,3);zeros(3,3) inv(alpha2.I)];

% Linkage beta-1

beta1.m = 142.168440;
beta1.I = [ 91843.636529	 -772.772852	 40829.647339 ;
            -772.772852    517848.015598	 -201.475342  ;
        	40829.647339	 -201.475342	 493681.548715];
beta1.beta1_B1 = [-75.5 0 -28.66]';
beta1.nx = [1 0 0]';
beta1.ny = [0 1 0]';
beta1.nz = [0 0 1]';
beta1.W = [(1/beta1.m)*eye(3) zeros(3,3);zeros(3,3) inv(beta1.I)];

% Linkage beta-2

beta2.m = 142.168440;
beta2.I = [ 91843.636529	  772.772852	 -40829.647339 ;
             772.772852    517848.015598	 -201.475342  ;
        	-40829.647339	 -201.475342	 493681.548715];
beta2.beta2_B2 = [-75.5 0 28.66]';
beta2.nx = [1 0 0]';
beta2.ny = [0 1 0]';
beta2.nz = [0 0 1]';
beta2.W = [(1/beta2.m)*eye(3) zeros(3,3);zeros(3,3) inv(beta2.I)];

% Linkage gamma-1

gamma1.m =  49.049581 ;
gamma1.I =[ 2956.157949	    0.004740	 1300.105736 ;
	        0.004740	     92320.757630	  -0.000375 ;
	        1300.105736	   -0.000375	  90182.095841];
gamma1.gamma1_A1 = [-65.68 0 -4.57]'; %[-65.68 5 -4.57]'; 
gamma1.nx = [1 0 0]';
gamma1.ny = [0 1 0]';
gamma1.nz = [0 0 1]';
gamma1.W = [(1/gamma1.m)*eye(3) zeros(3,3);zeros(3,3) inv(gamma1.I)];

% Linkage gamma-2

gamma2.m = 49.049581;
gamma2.I = [ 2956.157949	  -0.004740	   -1300.105736 ;
	        -0.004740	     92320.757630	  -0.000375 ;
	        -1300.105736	   -0.000375	  90182.095841];
gamma2.gamma2_A2 =  [-65.68 0 4.57]'; % [-65.68 -5 4.57]';
gamma2.nx = [1 0 0]';
gamma2.ny = [0 1 0]';
gamma2.nz = [0 0 1]';
gamma2.W = [(1/gamma2.m)*eye(3) zeros(3,3);zeros(3,3) inv(gamma2.I)];

% Linkage delta-1

delta1.m =  15.681471 ;
delta1.I = [  352.062649	 0.000000	    0.000000;
	            0.000000	 13277.939609	 0.000000;
	            0.000000	 0.000000	 13187.234818];
delta1.delta1_D1 =  [-42.6 0 0]'; % [-42.6 5 0]'; 
delta1.nx = [1 0 0]';
delta1.ny = [0 1 0]';
delta1.nz = [0 0 1]';
delta1.W = [(1/delta1.m)*eye(3) zeros(3,3);zeros(3,3) inv(delta1.I)];

% Linkage delta-2

delta2.m =   15.681471;
delta2.I = [  352.062649	 0.000000	    0.000000;
	            0.000000	 13277.939609	 0.000000;
	            0.000000	 0.000000	 13187.234818];
delta2.delta2_D2 =  [-42.6 0 0]'; % [-42.6 -5 0]'; 
delta2.nx = [1 0 0]';
delta2.ny = [0 1 0]';
delta2.nz = [0 0 1]';
delta2.W = [(1/delta2.m)*eye(3) zeros(3,3);zeros(3,3) inv(delta2.I)];

% Linkage epsilon-1

epsilon1.m =  66.713674;
epsilon1.I = [10744.530879   1691.358784	   735.827120 ;
	           1691.358784	  57872.558479	  779.660039 ;
	            735.827120	  779.660039	  58548.216359];
epsilon1.epsilon1_F1 = [-44.1 0 -7.6]';
epsilon1.nx = [1 0 0]';
epsilon1.ny = [0 1 0]';
epsilon1.nz = [0 0 1]';
epsilon1.W = [(1/epsilon1.m)*eye(3) zeros(3,3);zeros(3,3) inv(epsilon1.I)];

% Linkage epsilon-2

epsilon2.m =  66.713674;
epsilon2.I = [10744.530879  -1691.358784	  -735.827120 ;
	 -1691.358784	  57872.558479	  779.660039 ;
	  -735.827120	  779.660039	  58548.216359];
epsilon2.epsilon2_F2 = [-44.1 0 7.6]';
epsilon2.nx = [1 0 0]';
epsilon2.ny = [0 1 0]';
epsilon2.nz = [0 0 1]';
epsilon2.W = [(1/epsilon2.m)*eye(3) zeros(3,3);zeros(3,3) inv(epsilon2.I)];

% Combining the unit vectors for convinience
% ns = [alpha1.nx ... alpha2.nx ... beta1.nx ........epsilon2.nz O.nx ...]
% O is at last

ns = zeros(3,3,n);
for i = 1 : n
    ns(:,:,i) = eye(3);  % [ nx ny nz ] = eye(3)
end

I = [O.I alpha1.I alpha2.I beta1.I beta2.I gamma1.I gamma2.I delta1.I delta2.I epsilon1.I epsilon2.I];
M = [O.m alpha1.m alpha2.m beta1.m beta2.m gamma1.m gamma2.m delta1.m delta2.m epsilon1.m epsilon2.m];

L = length(17:0.1:160);
F_O1_O  = zeros(6,L);
F_O1_a1 = zeros(6,L);
F_O2_O  = zeros(6,L);
F_O2_a2 = zeros(6,L);
F_A1_g1 = zeros(6,L);
F_A1_a1 = zeros(6,L);
F_A2_g2 = zeros(6,L);
F_A2_a2 = zeros(6,L);
F_B1_a1 = zeros(6,L);
F_B1_b1 = zeros(6,L);
F_B2_a2 = zeros(6,L);
F_B2_b2 = zeros(6,L);
F_C1_d1 = zeros(6,L);
F_C1_b1 = zeros(6,L);
F_C2_d2 = zeros(6,L);
F_C2_b2 = zeros(6,L);
F_D1_g1 = zeros(6,L);
F_D1_d1 = zeros(6,L);
F_D2_g2 = zeros(6,L);
F_D2_d2 = zeros(6,L);
F_E_g2  = zeros(6,L);
F_E_g1  = zeros(6,L);
F_F1_b1 = zeros(6,L);
F_F1_e1 = zeros(6,L);
F_F2_b2 = zeros(6,L);
F_F2_e2 = zeros(6,L);
F_G_e2  = zeros(6,L);
F_G_e1  = zeros(6,L);
theta_index = 0;

%% Calculating the force at each configuration

for theta = 33.6 %17:0.1:160
    theta_index = theta_index + 1;
    k = 1;
    %% Initial Conditions and Setup
    
    % Since Sharon senpai's algorithm only contains kinematic informaions,
    % CoM positions in each local frames are required to calculate the neeeded configuration.
    relative_CoM_position = [     alpha1.alpha1_O       alpha2.alpha2_O     beta1.beta1_B1     beta2.beta2_B2 ...
                                  gamma1.gamma1_A1      gamma2.gamma2_A2   delta1.delta1_D1   delta2.delta2_D2 ...
                              epsilon1.epsilon1_F1  epsilon2.epsilon2_F2 ];
    
    [config,s0,G0] = conditions_setup(theta,relative_CoM_position,O.x,O.q);
    % s0 is the initial position/orientation vector
    
    % The state vector is constructed like this : [O.x O.q alpha1.x alpha1.q alpha2.x alpha2.q .....]'
    % alpha1.x : The position of C.O.M of alpha-1 in the world coordinate. [x y z]
    % alpha1.q : The orientation of alpha-1 in world coordinate. [q0 q1 q2 q3] (quaternions)
    % 7 states per object
    s = zeros(7*n,1);
    s(:,1) = s0;
    
    % Velocity / Angular velocity state vector [O.v O.w alpha1.v alpha1.w alpha2.v alpha2.w .....]'
    v = zeros(6*n,1);
    
    % The external wrench input of each link at each step is discreted
    wrench.ext  = zeros(6*n,1); % 3D force/torque for all n links.
    
    % non torque term of the euler's of motion
    wrench.w    = zeros(6*n,1);
    
    % Gravatational force
    wrench.g    = zeros(6,n);
    wrench.g(3,:) = M*(-g);
    wrench.g = reshape(wrench.g,6*n,1);
    
    % Reactional force factor (Largrange Multiplier)
    nc = 14; % Number of joints(hinge)
    lambda = zeros(5*nc,1);
    wrench.friction  = zeros(6*n,1);
    C = 0 ; % Torsional friction coefficient 
    wrench.react = zeros(6*n,1);
    wrench.total = zeros(6*n,1);
    J = zeros(5*nc,6*n,1);
    E = zeros(5*nc,1);
    
    %% Solver
    
    %% Rotate the inertia tensor
    I_rot = zeros(3,3*n);
    for i = 0 : n-1
        I_rot(:, 3*i + 1 : 3*i + 3  ) = Rq(s( 7*i + 4 : 7*i + 7 ,k)) * I(:,3*i+1:3*i+3)  * Rq(s( 7*i + 4 : 7*i + 7 ,k ))';
        I_rot(:, 3*i + 1 : 3*i + 3  ) = I_rot(:, 3*i + 1 : 3*i + 3  ) - tril(I_rot(:, 3*i + 1 : 3*i + 3  ),-1) + triu(I_rot(:, 3*i + 1 : 3*i + 3  ),1)'; 
    end
    
    %% Constructing the inverse mass matrix
    W = zeros(6*n,6*n);
    for i = 0 : n-1
        W( (6*i + 1) : (6*i + 6) , (6*i + 1) : (6*i + 6)) = [ 1/M(i+1)*eye(3) zeros(3,3) ; zeros(3,3) inv(I_rot(:, 3*i + 1 : 3*i + 3 ))];
    end

    %%  Refresh the wrenches
    
    % wrench.w
    for i = 0 : n-1
        w = v( 6*i + 4 : 6*i + 6 , k) ;
        wrench.w( 6*i + 4 : 6*i + 6 , k) = - skew(w) * I_rot(:, 3*i + 1 : 3*i + 3) * w;
    end
    % wrench.ext
    % 1 N   = 10^6 mm   * g / s^2
    % 1 N*m = 10^9 mm^2 * g / s^2
    
    wrench.ext( 1: 6,k) = [ 0  0  0   0  0  0 ]';
    % wrench.ext( 7:12,k) = [Rq(s(11:14,k)) zeros(3,3) ; zeros(3,3) Rq(s(11:14,k))] * [  0  0  0  0  1e9  0]';
    % wrench.ext(13:18,k) = [Rq(s(18:21,k)) zeros(3,3) ; zeros(3,3) Rq(s(18:21,k))] * [  0  0  0  0 -1e9  0]';
    % wrench.ext(43:48,k) = [  0  0  0   0  0  0]';
    % wrench.ext( 7:12,k) = [  0  0  -1e6   0   1e9  0]';
    % wrench.ext(13:18,k) = [  0  0  -1e6   0  -1e9  0]';
    G_Force = [0 0 400]'*1e6;
    G_e1G = Rq(s(67:70,k))*config(:,2, 9);
    G_e2G = Rq(s(74:77,k))*config(:,2,10);
    wrench.ext(55:57,k) =  (G_Force'*G_e2G / (G_Force'*G_e1G + G_Force'*G_e2G)) * G_Force;
  %  wrench.ext(58:60,k) = skew(G_e1G)*wrench.ext(55:57,k);
    wrench.ext(61:63,k) =  (G_Force'*G_e1G / (G_Force'*G_e1G + G_Force'*G_e2G)) * G_Force;
  %  wrench.ext(64:66,k) = skew(G_e2G)*wrench.ext(61:63,k);
    
%% Friction Wrench
    
    H1  = [ zeros(3,3) -eye(3) ; zeros(3,3)  eye(3) ];
    H2 = [ zeros(3,3)  eye(3) ; zeros(3,3) -eye(3) ];
    X = zeros(6,6);

    B = ...
      [H1  H2   X   X   X   X   X   X   X   X   X ;
       H1   X  H2   X   X   X   X   X   X   X   X ;
        X  H1   X   X   X  H2   X   X   X   X   X ;
        X   X  H1   X   X   X  H2   X   X   X   X ;
        X  H1   X  H2   X   X   X   X   X   X   X ;
        X   X  H1   X  H2   X   X   X   X   X   X ;
        X   X   X  H1   X   X   X  H2   X   X   X ;
        X   X   X   X  H1   X   X   X  H2   X   X ;
        X   X   X   X   X  H1   X  H2   X   X   X ;
        X   X   X   X   X   X  H1   X  H2   X   X ;
        X   X   X   X   X  H1  H2   X   X   X   X ;
        X   X   X  H1   X   X   X   X   X  H2   X ;
        X   X   X   X  H1   X   X   X   X   X  H2 ;
        X   X   X   X   X   X   X   X   X  H1  H2 ; ];
    
    S = B*v;
    wrench.friction( 1: 6,k) =  [ 0 0 0 (S( 1: 3)' + S( 7: 9)'            )]';
    wrench.friction( 7:12,k) =  [ 0 0 0 (S( 4: 6)' + S(16:18)' + S(25:27)')]';
    wrench.friction(13:18,k) =  [ 0 0 0 (S(10:12)' + S(22:24)' + S(31:33)')]';
    wrench.friction(19:24,k) =  [ 0 0 0 (S(28:30)' + S(40:42)' + S(67:69)')]';
    wrench.friction(25:30,k) =  [ 0 0 0 (S(34:36)' + S(46:48)' + S(73:75)')]';
    wrench.friction(31:36,k) =  [ 0 0 0 (S(13:15)' + S(49:51)' + S(64:66)')]';
    wrench.friction(37:42,k) =  [ 0 0 0 (S(19:21)' + S(55:57)' + S(61:63)')]';
    wrench.friction(43:48,k) =  [ 0 0 0 (S(37:39)' + S(52:54)'            )]';
    wrench.friction(49:54,k) =  [ 0 0 0 (S(43:45)' + S(58:60)'            )]';
    wrench.friction(55:60,k) =  [ 0 0 0 (S(70:72)' + S(82:84)'            )]';
    wrench.friction(61:66,k) =  [ 0 0 0 (S(76:78)' + S(79:81)'            )]';
    
    %% Construct the Constraint Jacobian J
    J(:,:,k) = The_Jacobian3(s(:,k),ns,config);
    
    %% Calculate the Reactional Force  
    v_ = v(:,k) + time.period * W * ( wrench.ext(:,k) + wrench.w(:,k) + wrench.g + wrench.friction(:,k)) ;
    K =  ( J(:,:,k) * W * J(:,:,k)' ) ;
    K = K - tril(K,-1) + triu(K,1)'; % Make sure K is still symmetric
    lambda(:,k) = - K \ (( J(:,:,k) * v_) / time.period);
    wrench.react(:,k) = J(:,:,k)'*lambda(:,k);
    wrench.total(:,k) = ( wrench.react(:,k) + wrench.ext(:,k) + wrench.w(:,k) + wrench.g + wrench.friction(:,k) );
    
    %% Analyze the joint forces from the Jacobian and the lambda (Largrange Multipliers)

    ns_ = zeros(3,3,n);
    ns_(:,:,n) = Rq(s(4:7,k))*ns(:,:,n);
    config_ = zeros(3,3,10);
    for i = 1:n-1
        ns_(:,:,i) = Rq(s( 7*i+4 : 7*i+7 ,k))*ns(:,:,i);
        config_(:,:,i) = Rq(s( 7*i+4 : 7*i+7 ,k))*config(:,:,i);
    end
   
    Fr = zeros(84*2,1);
    
    for i = 0 : 13
        Fr( 12*i + 4 : 12*i + 6 ) =  S ( 6*i + 1 : 6*i + 3) ;
        Fr( 12*i + 10 : 12*i + 12 ) = S ( 6*i + 4 : 6*i + 6) ;
    end
    
    % O1 O/a1
    J_O1_O         = [  eye(3)            zeros(3,3) ; zeros(1,3)   ns_(:, 1,11)' ; zeros(1,3)   ns_(: ,3,11)' ];
    J_O1_a1        = [ -eye(3)  skew(config_(:,1, 1)) ; zeros(1,3)  -ns_(:, 1,11)' ; zeros(1,3)  -ns_(:, 3,11)' ];

    F_O1_O (:,theta_index) = [ Rq(s( 4: 7,k))' zeros(3,3) ; zeros(3,3) Rq(s( 4: 7,k))' ] * ( J_O1_O'  * lambda(1:5,k) + Fr( 1: 6) );
    F_O1_a1(:,theta_index) = [ Rq(s(11:14,k))' zeros(3,3) ; zeros(3,3) Rq(s(11:14,k))' ] * ( J_O1_a1' * lambda(1:5,k) + Fr( 7:12) ) ;

    % O2 O/a2
    J_O2_O         = [  eye(3)            zeros(3,3) ; zeros(1,3)   ns_(:, 1,11)' ; zeros(1,3)   ns_(:, 3,11)' ];
    J_O2_a2        = [ -eye(3)  skew(config_(:,1, 2)) ; zeros(1,3)  -ns_(:, 1,11)' ; zeros(1,3)  -ns_(:, 3,11)' ];

    F_O2_O(:,theta_index)  = [ Rq(s(4:7,k))' zeros(3,3) ; zeros(3,3) Rq(s(4:7,k))' ] *( J_O2_O'  * lambda(6:10,k)+ Fr(13:18) );
    F_O2_a2(:,theta_index) = [ Rq(s(18:21,k))' zeros(3,3) ; zeros(3,3) Rq(s(18:21,k))' ] * (J_O2_a2' * lambda(6:10,k)+ Fr(19:24) );

    % A1 g1/a1
    J_A1_g1        = [  eye(3) -skew(config_(:,1, 5)) ; zeros(1,3)   ns_(:, 1, 5)' ; zeros(1,3)   ns_(:, 3, 5)' ];
    J_A1_a1        = [ -eye(3)  skew(config_(:,2, 1)) ; zeros(1,3)  -ns_(:, 1, 5)' ; zeros(1,3)  -ns_(:, 3, 5)' ];

    F_A1_g1(:,theta_index)  = [ Rq(s(39:42,k))' zeros(3,3) ; zeros(3,3) Rq(s(39:42,k))' ] * (J_A1_g1' * lambda(11:15,k)+ Fr(25:30) );
    F_A1_a1(:,theta_index)  = [ Rq(s(11:14,k))' zeros(3,3) ; zeros(3,3) Rq(s(11:14,k))' ] * (J_A1_a1' * lambda(11:15,k)+ Fr(31:36) );

    % A2 g2/a2
    J_A2_g2        = [  eye(3) -skew(config_(:,1, 6)) ; zeros(1,3)   ns_(:, 1, 6)' ; zeros(1,3)   ns_(:, 3, 6)' ];
    J_A2_a2        = [ -eye(3)  skew(config_(:,2, 2)) ; zeros(1,3)  -ns_(:, 1, 6)' ; zeros(1,3)  -ns_(:, 3, 6)' ];

    F_A2_g2(:,theta_index)  = [ Rq(s(46:49,k))' zeros(3,3) ; zeros(3,3) Rq(s(46:49,k))' ] *( J_A2_g2' * lambda(16:20,k) + Fr(37:42) );
    F_A2_a2(:,theta_index)  = [ Rq(s(18:21,k))' zeros(3,3) ; zeros(3,3) Rq(s(18:21,k))' ] *( J_A2_a2' * lambda(16:20,k) + Fr(43:48) );

    % B1 a1/b1
    J_B1_a1        = [  eye(3) -skew(config_(:,3, 1)) ; zeros(1,3)   ns_(:, 1, 1)' ; zeros(1,3)   ns_(:, 3, 1)' ];
    J_B1_b1        = [ -eye(3)  skew(config_(:,1, 3)) ; zeros(1,3)  -ns_(:, 1, 1)' ; zeros(1,3)  -ns_(:, 3, 1)' ];

    F_B1_a1(:,theta_index)  = [ Rq(s(11:14,k))' zeros(3,3) ; zeros(3,3) Rq(s(11:14,k))' ] *( J_B1_a1' * lambda(21:25,k)+ Fr(49:54) );
    F_B1_b1(:,theta_index)  = [ Rq(s(25:28,k))' zeros(3,3) ; zeros(3,3) Rq(s(25:28,k))' ]  *(J_B1_b1' * lambda(21:25,k)+ Fr(55:60) );

    % B2 a2/b2
    J_B2_a2        = [  eye(3) -skew(config_(:,3, 2)) ; zeros(1,3)   ns_(:, 1, 2)' ; zeros(1,3)   ns_(:, 3, 2)' ];
    J_B2_b2        = [ -eye(3)  skew(config_(:,1, 4)) ; zeros(1,3)  -ns_(:, 1, 2)' ; zeros(1,3)  -ns_(:, 3, 2)' ];

    F_B2_a2(:,theta_index)  = [ Rq(s(18:21,k))' zeros(3,3) ; zeros(3,3) Rq(s(18:21,k))' ] *( J_B2_a2' * lambda(26:30,k)+ Fr(61:66) );
    F_B2_b2(:,theta_index)  = [ Rq(s(32:35,k))' zeros(3,3) ; zeros(3,3) Rq(s(32:35,k))' ] *( J_B2_b2' * lambda(26:30,k)+ Fr(67:72) );

    % C1 d1/b1
    J_C1_d1        = [  eye(3) -skew(config_(:,2, 7)) ; zeros(1,3)   ns_(:, 1, 7)' ; zeros(1,3)   ns_(:, 3, 7)' ];
    J_C1_b1        = [ -eye(3)  skew(config_(:,2, 3)) ; zeros(1,3)  -ns_(:, 1, 7)' ; zeros(1,3)  -ns_(:, 3, 7)' ];

    F_C1_d1(:,theta_index)  = [ Rq(s(53:56,k))' zeros(3,3) ; zeros(3,3) Rq(s(53:56,k))' ] * ( J_C1_d1' * lambda(31:35,k)+ Fr(73:78) );
    F_C1_b1(:,theta_index)  = [ Rq(s(25:28,k))' zeros(3,3) ; zeros(3,3) Rq(s(25:28,k))' ] * ( J_C1_b1' * lambda(31:35,k)+ Fr(79:84) );

    % C2 d2/b2
    J_C2_d2        = [  eye(3) -skew(config_(:,2, 8)) ; zeros(1,3)   ns_(:, 1, 8)' ; zeros(1,3)   ns_(:, 3, 8)' ];
    J_C2_b2        = [ -eye(3)  skew(config_(:,2, 4)) ; zeros(1,3)  -ns_(:, 1, 8)' ; zeros(1,3)  -ns_(:, 3, 8)' ];

    F_C2_d2(:,theta_index)  = [ Rq(s(60:63,k))' zeros(3,3) ; zeros(3,3) Rq(s(60:63,k))' ] * (J_C2_d2' * lambda(36:40,k)+ Fr(85:90) );
    F_C2_b2(:,theta_index)  = [ Rq(s(32:35,k))' zeros(3,3) ; zeros(3,3) Rq(s(32:35,k))' ] * (J_C2_b2' * lambda(36:40,k)+ Fr(91:96) );

    % D1 g1/d1
    J_D1_g1        = [  eye(3) -skew(config_(:,2, 5)) ; zeros(1,3)   ns_(:, 1, 5)' ; zeros(1,3)   ns_(:, 3, 5)' ];
    J_D1_d1        = [ -eye(3)  skew(config_(:,1, 7)) ; zeros(1,3)  -ns_(:, 1, 5)' ; zeros(1,3)  -ns_(:, 3, 5)' ];

    F_D1_g1(:,theta_index)  = [ Rq(s(39:42,k))' zeros(3,3) ; zeros(3,3) Rq(s(39:42,k))' ] * ( J_D1_g1' * lambda(41:45,k)+ Fr(97:102) );
    F_D1_d1(:,theta_index)  = [ Rq(s(53:56,k))' zeros(3,3) ; zeros(3,3) Rq(s(53:56,k))' ] * ( J_D1_d1' * lambda(41:45,k)+ Fr(103:108) );

    % D2 g2/d2
    J_D2_g2        = [  eye(3) -skew(config_(:,2, 6)) ; zeros(1,3)   ns_(:, 1, 6)' ; zeros(1,3)   ns_(:, 3, 6)' ];
    J_D2_d2        = [ -eye(3)  skew(config_(:,1, 8)) ; zeros(1,3)  -ns_(:, 1, 6)' ; zeros(1,3)  -ns_(:, 3, 6)' ];

    F_D2_g2(:,theta_index)  =  [ Rq(s(46:49,k))' zeros(3,3) ; zeros(3,3) Rq(s(46:49,k))' ] * ( J_D2_g2' * lambda(46:50,k)+ Fr(109:114) );
    F_D2_d2(:,theta_index)  =  [ Rq(s(60:63,k))' zeros(3,3) ; zeros(3,3) Rq(s(60:63,k))' ] * ( J_D2_d2' * lambda(46:50,k)+ Fr(115:120) );

    % E  g2/g1
    J_E_g2         = [  eye(3) -skew(config_(:,3, 6)) ; zeros(1,3)   ns_(:, 1, 6)' ; zeros(1,3)   ns_(:, 3, 6)' ];
    J_E_g1         = [ -eye(3)  skew(config_(:,3, 5)) ; zeros(1,3)  -ns_(:, 1, 6)' ; zeros(1,3)  -ns_(:, 3, 6)' ];

    F_E_g2(:,theta_index)  = [ Rq(s(46:49,k))' zeros(3,3) ; zeros(3,3) Rq(s(46:49,k))' ] * ( J_E_g2' * lambda(51:55,k)+ Fr(121:126) );
    F_E_g1(:,theta_index)  = [ Rq(s(39:42,k))' zeros(3,3) ; zeros(3,3) Rq(s(39:42,k))' ] * ( J_E_g1' * lambda(51:55,k)+ Fr(127:132) );

    % F1 b1/e1
    J_F1_b1        = [  eye(3) -skew(config_(:,3, 3)) ; zeros(1,3)   ns_(:, 1, 3)' ; zeros(1,3)   ns_(:, 3, 3)' ];
    J_F1_e1        = [ -eye(3)  skew(config_(:,1, 9)) ; zeros(1,3)  -ns_(:, 1, 3)' ; zeros(1,3)  -ns_(:, 3, 3)' ];

    F_F1_b1(:,theta_index)  = [ Rq(s(25:28,k))' zeros(3,3) ; zeros(3,3) Rq(s(25:28,k))' ] * ( J_F1_b1' * lambda(56:60,k)+ Fr(133:138) );
    F_F1_e1(:,theta_index)  = [ Rq(s(67:70,k))' zeros(3,3) ; zeros(3,3) Rq(s(67:70,k))' ] * ( J_F1_e1' * lambda(56:60,k)+ Fr(139:144) );

    % F2 b2/e2
    J_F2_b2        = [  eye(3) -skew(config_(:,3, 4)) ; zeros(1,3)   ns_(:, 1, 4)' ; zeros(1,3)   ns_(:, 3, 4)' ];
    J_F2_e2        = [ -eye(3)  skew(config_(:,1,10)) ; zeros(1,3)  -ns_(:, 1, 4)' ; zeros(1,3)  -ns_(:, 3, 4)' ];

    F_F2_b2(:,theta_index)  = [ Rq(s(32:35,k))' zeros(3,3) ; zeros(3,3) Rq(s(32:35,k))' ] * ( J_F2_b2' * lambda(61:65,k)+ Fr(145:150) );
    F_F2_e2(:,theta_index)  = [ Rq(s(74:77,k))' zeros(3,3) ; zeros(3,3) Rq(s(74:77,k))' ] * ( J_F2_e2' * lambda(61:65,k)+ Fr(151:156) );

    % G  e2/e1
    J_G_e2         = [  eye(3) -skew(config_(:,2,10)) ; zeros(1,3)   ns_(:, 1,10)' ; zeros(1,3)   ns_(:, 3,10)' ];
    J_G_e1         = [ -eye(3)  skew(config_(:,2, 9)) ; zeros(1,3)  -ns_(:, 1,10)' ; zeros(1,3)  -ns_(:, 3,10)' ];

    F_G_e2(:,theta_index)  = [ Rq(s(74:77,k))' zeros(3,3) ; zeros(3,3) Rq(s(74:77,k))' ] * ( J_G_e2' * lambda(66:70,k) + Fr(157:162) );
    F_G_e1(:,theta_index)  = [ Rq(s(67:70,k))' zeros(3,3) ; zeros(3,3) Rq(s(67:70,k))' ] * ( J_G_e1' * lambda(66:70,k) + Fr(163:168) );

end