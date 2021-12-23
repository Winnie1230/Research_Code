clear all; close all; clc;

%% Linkleg parameters
ParamFile = 'param_R100.mat';
load(ParamFile);      % according to the size of the wheel
LinkArcFile = CalculateLinkArc(ParamFile);

O = [0;0];
shift = [0;0];
theta = 63;
beta = 0;
wheel_point = CalculateCoordinate2(LinkArcFile,theta0_deg);
leg_point = CalculateCoordinate2(LinkArcFile,theta);

wheel_GF_r = wheel_point.F1 - wheel_point.G;
leg_GF_r = leg_point.F1 - leg_point.G;

wheel_GF_l = wheel_point.F2 - wheel_point.G;
leg_GF_l = leg_point.F2 - leg_point.G;

angle_r = AngleBetween2Dvector(leg_GF_r,wheel_GF_r); %deg
angle_l = AngleBetween2Dvector(leg_GF_l,wheel_GF_l); %deg

%% rotation matrix
% actually R_l = R_r';
R_r = [cosd(angle_r) -sind(angle_r) 0;...
       sind(angle_r)  cosd(angle_r) 0;...
       0              0             1];

R_l = [cosd(angle_l)  -sind(angle_l) 0;...
       sind(angle_l)   cosd(angle_l) 0;...
       0               0             1];

% rotate through 180 deg to y axis(webots touchsensor cannot measure negative force)
Rts = [ cosd(180) 0 sind(180);...
       0          1 0        ;...
       -sind(180) 0 cosd(180)];

%% read data
data = readmatrix('output_63_bump_1kg_no_rotate.xlsx');
% data = readmatrix('output_63_bump_3kg_no_rotate.xlsx');

temp = data(:,end-2:end)';
correct_Fz_r = (R_r*data(:,end-5:end-3)')';
correct_Fz_l = (R_l*data(:,end-2:end)')';

trans_complete = (data(:,8) == 0);

correct_Fz_r(trans_complete,:) = zeros(length(find(trans_complete)),3);
correct_Fz_l(trans_complete,:) = zeros(length(find(trans_complete)),3);

figure(1);
subplot(3,1,1);
plot(data(:,1),data(:,9),'b');
title('contact','Interpreter','latex');
xlabel('t[s]','Interpreter','latex');

subplot(3,1,2);
plot(data(:,1),correct_Fz_r(:,2),'r');
title('Right touch sensor','Interpreter','latex');
xlabel('t[s]','Interpreter','latex');
ylabel('$F_y[N]$','Interpreter','latex');

subplot(3,1,3);
plot(data(:,1),data(:,4));
title('Right motor torque','Interpreter','latex');
xlabel('t[s]','Interpreter','latex');
ylabel('$\tau$[N]','Interpreter','latex');

figure(2);
subplot(3,1,1);
plot(data(:,1),data(:,9),'b');
title('contact','Interpreter','latex');
xlabel('t[s]','Interpreter','latex');

subplot(3,1,2);
plot(data(:,1),correct_Fz_l(:,2),'r');
title('Left touch sensor','Interpreter','latex');
xlabel('t[s]','Interpreter','latex');
ylabel('$F_y[N]$','Interpreter','latex');

subplot(3,1,3);
plot(data(:,1),data(:,7));
title('Left motor torque','Interpreter','latex');
xlabel('t[s]','Interpreter','latex');
ylabel('$\tau$[N]','Interpreter','latex');

% figure(3);
% plot(data(:,1),correct_Fz_r(:,2)+correct_Fz_l(:,2));
% title('Total $F_z$','Interpreter','latex');
% xlabel('t[s]','Interpreter','latex');
% ylabel('$\tau$[N]','Interpreter','latex');

% (correct_Fz_r(:,2)-correct_Fz_l(:,2));
% subplot(2,1,2);
% plot(data(:,1),correct_Fz_l(:,2));

%% Plot
figure;
subplot(1,3,1);
PlotLinkleg(ParamFile,leg_point.A1,leg_point.B1,leg_point.C1,leg_point.D1,leg_point.E,leg_point.F1,leg_point.G,O,shift,beta);
PlotQuiver(leg_point.G,correct_Fz_r(end,:),2,'#009100',2,'-'); hold on
title('right downframe $F_z$(N)','Interpreter','latex');
axis([-2*R 2*R -2*R R]);

subplot(1,3,2);
PlotLinkleg(ParamFile,leg_point.A1,leg_point.B1,leg_point.C1,leg_point.D1,leg_point.E,leg_point.F1,leg_point.G,O,shift,beta);
PlotQuiver(leg_point.G,correct_Fz_l(end,:),2,'b',2,'-'); hold on
title('left downframe $F_z$(N)','Interpreter','latex');
axis([-2*R 2*R -2*R R]);

subplot(1,3,3);
PlotLinkleg(ParamFile,leg_point.A1,leg_point.B1,leg_point.C1,leg_point.D1,leg_point.E,leg_point.F1,leg_point.G,O,shift,beta);
PlotQuiver(leg_point.G,correct_Fz_r(end,:)+correct_Fz_l(end,:),2,'r',2,'-'); hold on
title('total $F_z$(N)','Interpreter','latex');
axis([-2*R 2*R -2*R R]);

% figure(2);
% PlotLinkleg(ParamFile,leg_point.A1,leg_point.B1,leg_point.C1,leg_point.D1,leg_point.E,leg_point.F1,leg_point.G,O,shift,beta);
% PlotQuiver(leg_point.G,correct_Fz_r(end,:)+correct_Fz_l(end,:),2,'r',2,'-'); hold on
% title('total $F_z$(N)','Interpreter','latex');
% axis([-2*R 2*R -2*R R]);

% norm(correct_Fz1(end,:))
% norm(correct_Fz2(end,:))
norm(correct_Fz_r(end,:))+norm(correct_Fz_l(end,:))



