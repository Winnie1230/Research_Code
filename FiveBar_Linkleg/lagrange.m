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

%% matrices
T34 = @(theta3,theta4)[l3*sind(theta3) -l4*sind(theta4);...
                       l3*cosd(theta3) -l4*cosd(theta4)];

T12 = @(theta1,theta2)[l1*sind(theta1) -l2*sind(theta2);...
                       l1*cosd(theta1) -l2*cosd(theta2)];   

Cs12 = @(theta1,theta2)[lc1*sind(theta1) 0;...
                        0                lc2*sind(theta2)];

Cc12 = @(theta1,theta2)[lc1*cosd(theta1) 0;...
                        0               lc2*cosd(theta2)];

Cs34 = @(theta3,theta4)[lc3*sind(theta3) 0;...
                        0                lc4*sind(theta4)];

Cc34 = @(theta3,theta4)[lc3*cosd(theta3) 0;...
                        0                lc4*cosd(theta4)];

Hs12 = @(theta1,theta2)[l1*sind(theta1) 0;...
                        0               l2*sind(theta2)];

Hc12 = @(theta1,theta2)[l1*cosd(theta1) 0;...
                        0               l2*cosd(theta2)];

M12 = [m1 0 ; 0 m2];
M34 = [m3 0 ; 0 m4];
I12 = [I1 0 ; 0 I2];
I34 = [I3 0 ; 0 I4];

dT12dt = @(theta1,theta2,w1,w2)[l1*cosd(theta1)*w1 -l2*cosd(theta2)*w2;...
                                -l1*sind(theta1)*w1 l2*sind(theta2)*w2];

dT34dt = @(theta3,theta4,w3,w4)[l3*cosd(theta3)*]

%%
% ----- generalized 
D = @(theta1,theta2,theta3,theta4)(Cs12(theta1,theta2)'*M12*Cs12(theta1,theta2)+...
    Cc12(theta1,theta2)'*M12*Cc12(theta1,theta2)+...
    (Hs12(theta1,theta2)-Cs34(theta3,theta4)*inv(T34(theta3,theta4))*T12(theta1,theta2))'*M34*(Hs12(theta1,theta2)-Cs34(theta3,theta4)*inv(T34(theta3,theta4))*T12)+...
    (Hc12(theta1,theta2)-Cc34(theta3,theta4)*inv(T34(theta3,theta4))*T12(theta1,theta2))'*M34*(Hc12(theta1,theta2)-Cc34(theta3,theta4)*inv(T34(theta3,theta4))*T12)+...
    +I12+(inv(T34(theta3,theta4))*T12)'*I34*(inv(T34(theta3,theta4))*T12));

dDdt = 



