%% Solve Multibody System with LCP methods(Assume to be frictionless)
clear all; close all; clc;

% Linkage Order
% (1)Body

%% General Parameters (Using metic units - mmgs)

% Time
time.period = 0.001;
time.step = 2;

% Useful Constants
n = 1; % Number of linkages

%Environment
g = 9810 ;      % Gravitaional acceleration

% Constraints
no = 0; % Number of constraints to constrain Body to move vertically only
nc = 0; % Number of joints(hinge)
n_fix = 0; % Number of constraints to restrict the rotation around y-axis between alpha1 and O, alpha2 and O
% ball-and-socket joint only need a position constraint for translation
% 3 constraints for epsilon1 and 2 repectively
ngnd_fix = 0; % fix pointG to the ground(= regard pointG as Hinge Joint or Ball-And-Socket Joint?)

% plot
aa = 0.7;
bb = 0.7;
cc = 3;

% gif
del = 0.01; % time between animation frames

% create the video writer with 1 fps
% writerObj = VideoWriter('myVideo_fixed.avi');
% writerObj.FrameRate = 100;
% set the seconds per image
% open the video writer
% open(writerObj);

%% Linkage configurations (Using metic units - mmgs)
% Assume BASE is fixed to O(fix joint)

% ---------- Mass BASE(B) ----------
BASE.h = 100; % Height (Assume B is a solid cuboid)
BASE.w = 100; % Width
BASE.d = 100; % Depth
% B.x = [0 0 0]'; % Position of mass BASE
BASE.B_O = [0 0 -50]';
BASE.x = -BASE.B_O;

BASE.theta = 0;
BASE.q = [cos(BASE.theta/2) 0 0 sin(BASE.theta/2)]'; % Orientation of mass B (quaternion)
% B.m = 1000; % Mass
BASE.m = 1000; % Mass
BASE.I = BASE.m/12 * [ BASE.h^2+BASE.d^2 0 0 ; 0 BASE.w^2+BASE.d^2 0 ; 0 0 BASE.w^2+BASE.h^2 ]; % Inertia Tensor
BASE.W = [(1/BASE.m)*eye(3) zeros(3,3);zeros(3,3) inv(BASE.I)]; % Inverse inertia matrix

% sliding joint
BASE.anchor = [0 0 50];
% B.x_anchor = [0 0 -100]; % CoM to anchor point


% ---------- pin G(Rigid Body) ----------
GND.m = 1e+6; % 100kg
GND.h = 10; % Height (Assume B is a solid cuboid)
GND.w = 400; % Width
GND.d = 100; % Depth
GND.I = GND.m/12 * [ GND.h^2+GND.d^2 0 0 ; 0 GND.w^2+GND.d^2 0 ; 0 0 GND.w^2+GND.h^2 ]; % Inertia Tensor
GND.W = [(1/GND.m)*eye(3) zeros(3,3);zeros(3,3) inv(GND.I)]; % Inverse inertia matrix
GND.theta = 0;
GND.q = [cos(GND.theta/2) 0 0 sin(GND.theta/2)]'; % Orientation of mass B (quaternion)
GND.GND_G = [0 0 5]'; % local frame
GND.x = -GND.GND_G;
% ---------------------------------------

% Combining the unit vectors for convenience
% ns = [BASE.nx ... alpha1.nx ... alpha2.nx ... beta1.nx ........epsilon2.nz]
ns_ = zeros(3,3,n);
for i = 1 : n
    ns_(:,:,i) = eye(3);  % [ nx ny nz ] = eye(3)
end

M = BASE.m;
I = BASE.I;
% M = [BASE.m GND.m];
% I = [BASE.I GND.I];

vector.BASE_G = BASE.B_O;
vector.GND_G = GND.GND_G;

%% Book-keeping
history.s = zeros(7*n,time.step+1);
history.v = zeros(6*n,time.step+1);
history.J = zeros(5*nc + no + n_fix + ngnd_fix,6*n,time.step+1);
history.lambda = zeros(5*nc + no + n_fix + ngnd_fix,time.step+1);

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
% wrench.ext(11,1) = 16.708*2.41338*1e+9;
% wrench.ext(17,1) = -16.708*2.41338*1e+9;
% wrench.ext(5,1) = 1e+9;

% non torque term of the euler's of motion
wrench.w = zeros(6*n,1);

% Gravitational force
wrench.g = zeros(6,n);
wrench.g(3,:) = M*(-g);
wrench.g = reshape(wrench.g,6*n,1);

% Reactional force factor (Largrange Multiplier)
lambda = zeros(5*nc + no + n_fix + ngnd_fix,1);
% lambda = zeros(5*nc + no,1);
wrench.react = zeros(6*n,1);
wrench.total = zeros(6*n,1);

%% Initial Conditions and Setup / Initialize book-keeping

s0 = [ BASE.x ; BASE.q];
% s0 = [ BASE.x ; BASE.q ; GND.x ; GND.q];
v0 = zeros(6*n,1);
s = s0;
v = v0;

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

    %% Refresh wrenches
    for i = 0 : n-1
        w = v(6*i+4:6*(i+1),1);
        wrench.w(6*i+4:6*(i+1),1) = -skew(w) * I_rot(:,3*i+1 : 3*(i+1)) * w;
    end

    %% Construct the Jacobian matrices
    % rotate ns_
    ns = zeros(3,3,n);
    for i = 0 : n-1
        ns(:,:,i+1) = Rq(s(7*i+4 : 7*i+7,1))*ns_(:,:,i+1);
    end

    % Restrict BASE(B) to vertical movement only(sliding)
    % moving axis: z-dir
%     Js_BASE_trans = [ns(:,1,1)'   (skew(s(1:3,1))*ns(:,1,1)+2*skew(B.anchor-s(1:3,1))*ns(:,1,1))' ; ns(:,2,1)'   (skew(s(1:3))*ns(:,2,1)+2*skew(B.anchor-s(1:3,1))*ns(:,2,1))'];
    Js_BASE_trans = [[1;0;0]'   1/2*(skew(-s(1:3,1))*[1;0;0])' ; [0;1;0]'   1/2*(skew(-s(1:3,1))*[0;1;0])']; % Pickl_MT_2009
    Js_BASE_rot   = [zeros(3,3)   eye(3)];
    Js_BASE = [Js_BASE_trans ; Js_BASE_rot];

    % GND CONSTRAINTS
    J_fix_GND = [eye(3)  zeros(3,3) ; zeros(3,3) eye(3)];

    %% Bilateral Jacobian
    X = zeros(5, 6);
    x = zeros(1, 6);
    Z = zeros(6, 6);

%     Jb = [Z  J_fix_GND]; 
    Jb = [];
%     w0b = zeros(5*nc+ngnd_fix+n_fix+no,1);
    w0b = [];

    %% Unilateral Constraints
    %  End effect CONSTRAINTS (contact constraint)
    
    % wn: difference between velocity components in constraint-space and the target velocities of the constraints functions
    % w0n: target velocities can be used to obtain a nonzero relative
    % velocities between the bodies(assume to be zero)
    
    contact_normal = [0 0 1]'; % z-dir
    % epsilon1(10)
    Jc_G_BASE   = [ contact_normal'   (skew(vector.BASE_G)*contact_normal)'];

    % Jacobian
    Jn = [Jc_G_BASE];

    % wn
    wn = Jn*v;
    w0n = 0;

    %% Constraint violation(error)
    % Phi: constraints in position-level
    % Fix GND
%     Phi.fGND_trans = s(8:10) + vector.GND_G - [0;0;0];
%     Phi.fGND_rot = quat2eul(s(11:14,1)')' - [0;0;0];

%     Phi.bi_ERR = [Phi.fGND_trans ; Phi.fGND_rot];

    % epsilon1 and GND (contact)
    Phi.epsilon1_contact = (s(1:3) + vector.BASE_G)'*contact_normal;

    Phi.uni_ERR = Phi.epsilon1_contact;

    %% Regulartization
    % Unit transform
    % 1 N/m = 10^3 g mm/s^2 /mm;

    % The parameter is selected with reference to Numerical Solution of Mixed Linear Complementarity Problems in Multibody Dynamics with contact
    % bilateral
%     para.bi.k = 1e+10 * 1e+3; % stiffness
%     para.bi.b = 1e+8 * 1e+3; % damping
%     
%     % unilateral
%     para.uni.k = 1e+5 * 1e+3; % stiffness
%     para.uni.b = 1e+4 * 1e+3; % damping
% 
%     % stiffness matrix K
%     para.bi.K = para.bi.k*eye(size(Jb,1));
%     para.bi.B = para.bi.b*eye(size(Jb,1));
% 
%     % damping matrix B
%     para.uni.K = para.uni.k*eye(size(Jn,1));
%     para.uni.B = para.uni.b*eye(size(Jn,1));
% 
%     % matrix C
%     % C = (h*B+h^2*K)^-1
%     temp = time.period * para.bi.B + time.period^2 * para.bi.K;
%     para.bi.C = inv(time.period * para.bi.B + time.period^2 * para.bi.K);
%     para.uni.C = inv(time.period * para.uni.B + time.period^2 * para.uni.K);
% 
%     % vector d
%     % d = (h*eye() + BK^-1)^-1*Phi;
%     % inverse matrix calculation A\b for inv(A)*b or b/A for b*inv(A)
%     para.bi.d = (time.period * eye(size(Jb,1)) + para.bi.B/para.bi.K)\Phi.bi_ERR;
%     para.uni.d = (time.period * eye(size(Jn,1)) + para.uni.B/para.uni.K)\Phi.uni_ERR;

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
    Y = zeros(size(G,1),size(G,1));
    b_bar = -MassMatrix*v + time.period * wrench.w - time.period * (wrench.ext+wrench.g);
%     by = [b_bar ; para.bi.d - w0b];
    by = [b_bar ;  -w0b];
%     bx = para.uni.d - w0n;
    bx = -w0n;

    % linear complementarity problem(Ax+b = w, x>=0, w>=0, x'w=0)
    LCP.A = G*(H\G') + Y;
    LCP.b = -G*(H\by) + bx;

    [LCP.w,LCP.x,retcode] = LCPSolve(LCP.A,LCP.b);

    y = H\(G'*LCP.x - by);

    %% Calculate Reactional force
    wrench.react = Jn'*LCP.x/time.period;
    wrench.total = wrench.ext + wrench.g + wrench.react;
   
    Fn = Jn'*LCP.x/time.period;
    Fg = sum(wrench.g);

    %% Refresh system states
%     v = v + time.period * W * wrench.total;
% %     s = s + time.period * S * v;

    v = y(1:6*n,1);

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


    %% Refresh points and vectors pointing from CoM to Hinge Joints
    vector.BASE_O = Rq(s(4:7,1))*vector.BASE_G;
%     vector.GND_G = Rq(s(11:14,1))*vector.GND_G;

    %% Plot
    PlotCuboid1(s(1:3,1),BASE.w,BASE.d,BASE.h,Rq(s(4:7,1))  ,'#EAC100','#977C00'); hold on
%     PlotCuboid1(GND.x,GND.w,GND.d,GND.h,Rq(s(11:14)),'#EAC100','#977C00');

   % plot forces
    scale = 1e-5;
%     arrow3d([s(64,1)+vector.epsilon1_G(1) s(64,1)+vector.epsilon1_G(1)+scale*Fcj_G1_e1(1)],[s(65,1)+vector.epsilon1_G(2) s(65,1)+vector.epsilon1_G(2)+scale*Fcj_G1_e1(2)],[s(66,1)+vector.epsilon1_G(3) s(65,1)+vector.epsilon1_G(3)+scale*Fcj_G1_e1(3)],a,b,c,'k');
%     arrow3d([s(71,1)+vector.epsilon2_G(1) s(71,1)+vector.epsilon2_G(1)+scale*Fcj_G2_e2(1)],[s(72,1)+vector.epsilon2_G(2) s(72,1)+vector.epsilon2_G(2)+scale*Fcj_G2_e2(2)],[s(73,1)+vector.epsilon2_G(3) s(73,1)+vector.epsilon2_G(3)+scale*Fcj_G2_e2(3)],a,b,c,'k');
%     arrow3d([s(78,1) s(78,1)+scale*Fbs_G_e1(1)],[s(79,1) s(79,1)+scale*Fbs_G1_e1(2)],[s(80,1) s(80,1)+scale*Fbs_G_e1(3)],a,b,c,'#5B00AE');
%     arrow3d([s(78,1) s(78,1)+scale*Fbs_G_e2(1)],[s(79,1) s(79,1)+scale*Fbs_G2_e2(2)],[s(80,1) s(80,1)+scale*Fbs_G_e2(3)],a,b,c,'#007500');
    
%     PlotCuboid1(s(78:80,1),pinG.w,pinG.d,pinG.h,Rq(s(81:84,1))  ,'#EAC100','#977C00');
    hold off

    axis equal
    axis([-200 200 -200 200 -50 420]);
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
    history.J(:,:,t+1) = Jb;
    history.lambda(:,t+1) = lambda;
    history.wrench.ext(:,t+1) = wrench.ext;
    history.wrench.w(:,t+1) = wrench.w;
    history.wrench.g(:,t+1) = wrench.g;
    history.wrench.react(:,t+1) = wrench.react;
    history.wrench.total(:,t+1) = wrench.total;

end



