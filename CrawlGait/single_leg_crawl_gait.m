clear all; close all; clc;
% view leg motion from world frame

%% Linkleg parameters
ParamFile = 'param_R100.mat';
load(ParamFile);      % according to the size of the wheel
LinkArcFile = CalculateLinkArc(ParamFile);

%% OG to theta(mapping)
load('Theta_Length.mat');

%% Crawl gait parameters
r = 100; %mm
T = 8; %sec
dt = 1; %sec
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
O_shift_dx = stance_dx(2)-stance_dx(1);

% plot(traj_rf(:,1),traj_rf(:,2),'k-'); hold on
% plot(O(1),O(2),'ko','MarkerSize',10);
% axis equal;

theta_traj = zeros(length(swing_traj),1);
beta_traj = zeros(length(swing_traj),1);
diff_G = zeros(length(swing_traj),1);

figure
% del = 0.01; % time between animation frames

for run = 1:1
    traj(:,1) = traj(:,1) - 2*r*(run-1);
    actual_G = [r-2*r*(run-1);0];
    shift = [0-2*r*(run-1);sqrt(max_length^2-(2*r)^2)];
    O_shift_index = 1;
    swing_count = 0;
    touchdown_count = 0;
    liftoff_count = 0;
    stance_count = 0;

for i = 1:8
    clf();
    if(traj(i,2) ~= 0) % swing phase(3 leg landed)
        disp("swing_phase");
        
        des_G = traj(i,:)'; %desire G
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
        
        swing_count = swing_count + 1;
        
    else % stance phase/touchdown/lift off
        % touchdown/lift off(4 leg landed): don't shift O
        if(traj(i,1) == r-2*r*(run-1)) % lift off
            disp("lift off");
            actual_G = traj(i,:)';
            liftoff_count = liftoff_count + 1;
            
        elseif(traj(i,1) == -r-2*r*(run-1)) % touchdown
            disp("touchdown");
            actual_G = traj(i,:)';
            touchdown_count = touchdown_count + 1;
            
        else %stance
            disp("stance")
            shift(1) = (-2*r*(run-1))-O_shift_index*O_shift_dx;
            O_shift_index = O_shift_index + 1;
            
            stance_count = stance_count + 1;
        end
        
        fix_G = actual_G;
        OG = norm(fix_G-(O+shift));
        [row,col] = find(leg_length < OG);
        theta_deg = OGmap_theta(max(row),1);
        normal_OG = ((O+shift)-fix_G)/OG;
        
        % determine beta direction
        u = [normal_OG;0]; %turn normal_OG to 3D coordinate
        v = [0;1;0];
        uvcross = cross(v,u);
        sgn = -sign(uvcross(3));
        
        CosTheta = max(min(dot(normal_OG,[0;1])/(norm(normal_OG)*norm([0;1])),1),-1);
        beta_deg = sgn*real(acosd(CosTheta));
        [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,theta_deg);
    end
    % plot linkleg
    [Oup, up] = upframe_coordinate(R,up_deg,B,C,F);
    [Odn, dn] = downframe_coordinate(R,up_deg,F,G);
    [yc] = masscenter_coordinate(up,dn); %calculate the centroid
    
    [AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,~,FL,~,~,upR,upL,~,OdnL,dnR,dnL,masscenter] = rotate_coordinate(O,A,B,C,D,E,F,Oup,up,Odn,dn,yc,beta_deg);
    [O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter] = shift_coordinate(O,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter,shift);
    plot_linkleg2(O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter); hold on
    plot(traj(:,1),traj(:,2),'k-'); hold on
    plot(actual_G(1),actual_G(2),'bo','MarkerFaceColor','b');

    xlim([-400 250]);
    ylim([0 360]);
%     axis equal tight;
    drawnow;
    
    % ----- save to gif -----
%     frame = getframe(1);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     filename = ['crawl_gait1.gif'];
%     if (i == 1 && run == 1)
%         imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',del);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',del);
%     end
    % -----------------------
end
end




