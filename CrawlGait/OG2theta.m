clear all; close all; clc;
%% Linkleg parameters
ParamFile = 'param_R100.mat';
load(ParamFile);      % according to the size of the wheel
LinkArcFile = CalculateLinkArc(ParamFile);

%% OG to theta(function)
% inverse kinematics is too difficult to solve
OGmap_theta = (theta0_deg : 0.01 : th_hmax)';
leg_length = zeros(length(OGmap_theta),1);

for i = 1:length(OGmap_theta)
    theta_deg = OGmap_theta(i);
    [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,theta_deg);
    leg_length(i,1) = abs(G(2));
end
th_length = [OGmap_theta,leg_length];
filename = ['Theta_Length.mat'];
save(filename,'OGmap_theta','leg_length');
% plot(theta,leg_length,'b');