clear all; close all; clc;

center = [0 0 0];
l = 20; % mm
w = 10; % mm
h = 5; % mm
M = 8; %g
leg_len = 7.5; % mm(assume massless)
joint = [center(1)+w/2 center(2)+l/4 center(3);...
         center(1)-w/2 center(2)+l/4 center(3);...
         center(1)+w/2 center(2)-l/4 center(3);...
         center(1)-w/2 center(2)-l/4 center(3)];

end_eff = joint; end_eff(:,3) = end_eff(:,3)-leg_len;
% vert = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1];
vert = [center(1)-w/2 center(2)-l/2 center(3)-h/2;...
        center(1)+w/2 center(2)-l/2 center(3)-h/2;...
        center(1)+w/2 center(2)+l/2 center(3)-h/2;...
        center(1)-w/2 center(2)+l/2 center(3)-h/2;...
        center(1)-w/2 center(2)-l/2 center(3)+h/2;...
        center(1)+w/2 center(2)-l/2 center(3)+h/2;...
        center(1)+w/2 center(2)+l/2 center(3)+h/2;...
        center(1)-w/2 center(2)+l/2 center(3)+h/2];
fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 5 6 7 8; 1 2 3 4];

% patch('Vertices',vert,'Faces',fac,...
%       'FaceVertexCData',hsv(6),'FaceColor','flat')
plot3(center(1),center(2),center(3),'ro','MarkerSize',7,'MarkerFaceColor','r'); hold on
plot3(joint(:,1),joint(:,2),joint(:,3),'bo','MarkerSize',5,'MarkerFaceColor','b'); hold on
plot3(end_eff(:,1),end_eff(:,2),end_eff(:,3),'go','MarkerSize',5,'MarkerFaceColor','g'); hold on
plot3([joint(1,1),end_eff(1,1)],[joint(1,2),end_eff(1,2)],[joint(1,3),end_eff(1,3)],'k','LineWidth',2); hold on
plot3([joint(2,1),end_eff(2,1)],[joint(2,2),end_eff(2,2)],[joint(2,3),end_eff(2,3)],'k','LineWidth',2); hold on
plot3([joint(3,1),end_eff(3,1)],[joint(3,2),end_eff(3,2)],[joint(3,3),end_eff(3,3)],'k','LineWidth',2); hold on
plot3([joint(4,1),end_eff(4,1)],[joint(4,2),end_eff(4,2)],[joint(4,3),end_eff(4,3)],'k','LineWidth',2); hold on
patch('Vertices',vert,'Faces',fac,'FaceVertexCData',[1 0 0],'FaceColor',[1 1 1],'FaceAlpha', 0.5);

view(3)
axis vis3d
axis equal
xlim([-10 10])
ylim([-20 20])
zlim([-10 5])