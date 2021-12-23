clear all; close all; clc
%% Linkleg parameters
ParamFile = 'param_R100.mat';
load(ParamFile);      % according to the size of the wheel
LinkArcFile = CalculateLinkArc(ParamFile);

%% OG to theta(mapping)
Theta_Length = 'Theta_Length.mat';
load(Theta_Length);

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
traj_rf = [swing_traj;stance_traj];%1
traj_rh = [stance_traj;swing_traj];%4
traj_lf = [stance_traj(3/4*(T/dt)*1/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*1/3,:)];%3
traj_lh = [stance_traj(3/4*(T/dt)*2/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*2/3,:)];%2

% [stance_traj(3/4*(T/dt)*2/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*2/3,:)]
% [stance_traj(3/4*(T/dt)*1/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*1/3,:)]
% [stance_traj;swing_traj]
%% Maximum leg length
[A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,th_hmax);
max_length = norm(G);

O = [0;0];
ratio = 0.95; %ratio of maximum leg length
shift = [0;ratio*sqrt(max_length^2-r^2)];

%% Generate motor trajectory(degree)
[RF_theta,RF_beta,RF_phi_traj,RF_G_traj] = GenTrajectory(traj_rf,Theta_Length,O,shift,LinkArcFile); %RF
[LF_theta,LF_beta,LF_phi_traj,LF_G_traj] = GenTrajectory(traj_lf,Theta_Length,O,shift,LinkArcFile); %LF
[LH_theta,LH_beta,LH_phi_traj,LH_G_traj] = GenTrajectory(traj_lh,Theta_Length,O,shift,LinkArcFile); %LF
[RH_theta,RH_beta,RH_phi_traj,RH_G_traj] = GenTrajectory(traj_rh,Theta_Length,O,shift,LinkArcFile); %RH

%% convert degree to rad
RF_motor = RF_phi_traj;
RF_motor(:,1) = RF_motor(:,1) - theta0_deg; %right motorbar minus initial phi value
RF_motor(:,2) = RF_motor(:,2) + theta0_deg;
RF_phi_rad = RF_motor*pi/180;

LF_motor = LF_phi_traj;
LF_motor(:,1) = LF_motor(:,1) - theta0_deg;
LF_motor(:,2) = LF_motor(:,2) + theta0_deg;
LF_phi_rad = LF_motor*pi/180;

LH_motor = LH_phi_traj;
LH_motor(:,1) = LH_motor(:,1) - theta0_deg;
LH_motor(:,2) = LH_motor(:,2) + theta0_deg;
LH_phi_rad = LH_motor*pi/180;

RH_motor = RH_phi_traj;
RH_motor(:,1) = RH_motor(:,1) - theta0_deg;
RH_motor(:,2) = RH_motor(:,2) + theta0_deg;
RH_phi_rad = RH_motor*pi/180;

% ----- single leg mat file -----
% filename = ['R',num2str(R),'_crawl_singleleg.mat'];
% save(filename,'RF_phi_rad');

% ----- four leg mat file -----
filename = ['R',num2str(R),'_crawl_4leg.mat'];
save(filename, 'RF_phi_rad', 'RH_phi_rad', 'LF_phi_rad', 'LH_phi_rad');

%% Plot
figure('units','normalized','outerposition',[0 0 1 1])
% del = 0.01; % time between animation frames
for i = 1:1
    clf();
    subplot(2,2,1);
    xlim([-250 250]); ylim([0 360]);
    title('RF');
    [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,RF_theta(i,1));
    PlotLinkleg(ParamFile,A,B,C,D,E,F,G,O,shift,RF_beta(i,1),traj_rf,RF_G_traj(i,:));
    
    subplot(2,2,2);
    xlim([-250 250]); ylim([0 360]);
    title('RH');
    [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,RH_theta(i,1));
    PlotLinkleg(ParamFile,A,B,C,D,E,F,G,O,shift,RH_beta(i,1),traj_rh,RH_G_traj(i,:));

    subplot(2,2,3);
    xlim([-250 250]); ylim([0 360]);
    title('LF');
    [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,LF_theta(i,1));
    PlotLinkleg(ParamFile,A,B,C,D,E,F,G,O,shift,LF_beta(i,1),traj_lf,LF_G_traj(i,:));

    subplot(2,2,4);
    xlim([-250 250]); ylim([0 360]);
    title('LH');
    [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,LH_theta(i,1));
    PlotLinkleg(ParamFile,A,B,C,D,E,F,G,O,shift,LH_beta(i,1),traj_lh,LH_G_traj(i,:));
    
    drawnow;
    % ----- save to gif -----
%     frame = getframe(1);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     filename = ['4_leg_crawl_gait.gif'];
%     if (i == 1)
%         imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',del);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',del);
%     end
    % -----------------------
end
