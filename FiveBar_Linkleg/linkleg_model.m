clear all; close all; clc;

%% parameters
l0 = 60; %mm
l1 = 60; %mm
l2 = l1;
l3 = 145; %mm
l4 = l3;

w = 30; %linkage width(mm)
d = 5; %linkage thickness(mm)
rho = 2.7/1000; % Al density(g/mm^3)

m1 = rho*(l1*w*d); %g
m2 = rho*(l2*w*d); %g
m3 = rho*(l3*w*d); %g
m4 = rho*(l4*w*d); %g

lc1 = 1/2*l1;
lc2 = 1/2*l2;
lc3 = 1/2*l3;
lc4 = 1/2*l4;

I1 = 1/12*m1*l1^2;
I2 = 1/12*m2*l2^2;
I3 = 1/12*m3*l3^2;
I4 = 1/12*m4*l4^2;

M = 100; %g
g = 9.8*1000; %mm/s^2

% ----- save parameters -----
save('param.mat','l0','l1','l2','l3','l4','m1','m2','m3','m4','I1','I2','I3','I4','M','lc1','lc2','lc3','lc4','g');

theta1 = 225; %deg
theta2 = 315; %deg

theta3 = CalTheta3(theta1,theta2); %deg
theta4 = CalTheta4(theta1,theta2); %deg

% ----- check if modified answer is still an answer of the equation -----
ans1 = l1*cosd(theta1)+l3*cosd(theta3)-l2*cosd(theta2)-l4*cosd(theta4)-l0;
ans2 = l1*sind(theta1)+l3*sind(theta3)-l2*sind(theta2)-l4*sind(theta4);

%% plot
O = [0;0];
R1 = [cosd(theta1) -sind(theta1) ; sind(theta1) cosd(theta1)];
r1  = R1*[1;0];

R2 = [cosd(theta2) -sind(theta2) ; sind(theta2) cosd(theta2)];
r2 = R2*[1;0];

R3 = [cosd(theta3) -sind(theta3) ; sind(theta3) cosd(theta3)];
r3 = R3*[1;0];

R4 = [cosd(theta4) -sind(theta4) ; sind(theta4) cosd(theta4)];
r4 = R4*[1;0];

A = O + 1/2*l0*[-1;0];
B = A + l1*r1;
G = B + l3*r3;
D = O + 1/2*l0*[1;0];
C = D + l2*r2;

plot([O(1),A(1)],[O(2),A(2)],'k','LineWidth',2); hold on
plot([A(1),B(1)],[A(2),B(2)],'b','LineWidth',2); hold on
plot([B(1),G(1)],[B(2),G(2)],'r','LineWidth',2); hold on
plot([O(1),D(1)],[O(2),D(2)],'k','LineWidth',2); hold on
plot([D(1),C(1)],[D(2),C(2)],'b','LineWidth',2); hold on
plot([C(1),G(1)],[C(2),G(2)],'r','LineWidth',2); hold on

plot([O(1),O(1)],[O(2),O(2)],'k.','MarkerSize',20); hold on
plot([A(1),A(1)],[A(2),A(2)],'k.','MarkerSize',20); hold on
plot([B(1),B(1)],[B(2),B(2)],'k.','MarkerSize',20); hold on
plot([G(1),G(1)],[G(2),G(2)],'k.','MarkerSize',20); hold on
plot([D(1),D(1)],[D(2),D(2)],'k.','MarkerSize',20); hold on
plot([C(1),C(1)],[C(2),C(2)],'k.','MarkerSize',20); hold on

axis equal
% axis([-80 80 -120 0])

%% check length
check_l1 = norm(A-B);
check_l3 = norm(G-B);
check_l2 = norm(C-D);
check_l4 = norm(G-C);
