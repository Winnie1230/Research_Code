clear all; close all; clc;
%% Linkleg parameters
ParamFile = 'param_R100.mat';
load(ParamFile);      % according to the size of the wheel
LinkArcFile = CalculateLinkArc(ParamFile);

%% OG to theta(mapping)
load('Theta_Length.mat');

%% Crawl gait parameters
r = 100; %mm
T = 8; %sec
dt = 0.1; %sec
swing = 8/4; %swing phase time
stance = 8/4*3; % stance phase time

swing_ang = linspace(0,pi,swing/dt+1)';
swing_traj = [r*cos(swing_ang(1:end-1)), r*sin(swing_ang(1:end-1))];
stance_dx = linspace(-r,r,stance/dt+1)';
stance_traj = [stance_dx(1:end-1),zeros(length(stance_dx)-1,1)];
traj_rf = [swing_traj;stance_traj];
traj_lf = [stance_traj;swing_traj];
traj_lh = [stance_traj(3/4*(T/dt)*2/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*2/3,:)];
traj_rh = [stance_traj(3/4*(T/dt)*1/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*1/3,:)];

traj = traj_rf;

%% Maximum leg length
[A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,th_hmax);
max_length = norm(G);

O = [0;0];
ratio = 0.95; %ratio of maximum leg length
shift = [0;ratio*sqrt(max_length^2-r^2)];

theta_traj = zeros(length(traj),1);
beta_traj = zeros(length(traj),1);
diff_G = zeros(length(traj),1);

figure
% del = 0.01; % time between animation frames

for run = 1:1
    for i = 40
        clf();

        des_G = traj(i,:)';
        OG = norm(des_G-(O+shift));
        [row,col] = find(leg_length < OG);
        theta_deg = OGmap_theta(max(row),1);
        theta_traj(i,1) = theta_deg;
        normal_OG = ((O+shift)-des_G)/OG;

        % determine beta direction
        u = [normal_OG;0]; %turn normal_OG to 3D coordinate
        v = [0;1;0];
        uvcross = cross(v,u);
        sgn = -sign(uvcross(3));

        CosTheta = max(min(dot(normal_OG,[0;1])/(norm(normal_OG)*norm([0;1])),1),-1);
        beta_deg = sgn*real(acosd(CosTheta));
        beta_traj(i,1) = beta_deg;
        [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,theta_deg);
        actual_G = [cosd(beta_deg),sind(beta_deg);-sind(beta_deg),cosd(beta_deg)]*G';
        actual_G = actual_G + (O+shift);
        diff_G(i,1) = norm(des_G-actual_G);

        % plot linkleg
        [Oup, up] = upframe_coordinate(R,up_deg,B,C,F);
        [Odn, dn] = downframe_coordinate(R,up_deg,F,G);
        [yc] = masscenter_coordinate(up,dn); %calculate the centroid

        [AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,~,FL,~,~,upR,upL,~,OdnL,dnR,dnL,masscenter] = rotate_coordinate(O,A,B,C,D,E,F,Oup,up,Odn,dn,yc,beta_deg);
        [O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter] = shift_coordinate(O,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter,shift);
        plot_linkleg2(O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter); hold on
        plot(traj(:,1),traj(:,2),'k-'); hold on
        plot(actual_G(1),actual_G(2),'bo','MarkerFaceColor','b');
        
        plot([O_real(1),actual_G(1)],[O_real(2),actual_G(2)],'b');
        
        
        xlim([-250 250]);
        ylim([0 360]);
        drawnow;
    end
end

%%
% figure
% nbins = 100;
% 
% [count,edges] = histcounts(diff_G, nbins);
% histogram('BinCounts', count, 'BinEdges', edges);

%% convert theta and beta to phi_r and phi_l
phi = ([1 1;-1 1]*[theta_traj';beta_traj'])';

phi_r = phi(:,1) - theta0_deg;
phi_l = phi(:,2) + theta0_deg;

phi_r_rad = phi_r*pi/180;
phi_l_rad = phi_l*pi/180;


