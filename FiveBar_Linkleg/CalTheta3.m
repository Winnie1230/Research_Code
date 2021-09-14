function [theta3,theta4] = CalTheta3(theta1,theta2)
    load('param.mat');
    a = l1*cosd(theta1)-l2*cosd(theta2)-l0;
    b = l1*sind(theta1)-l2*sind(theta2);

    C1 = l4^2-(a^2+b^2+l3^2);
    C2 = 2*l3*sqrt(a^2+b^2);
    C3 = l3^2-(a^2+b^2+l4^2);
    C4 = 2*l4*sqrt(a^2+b^2);

    t3 = asind(C1/C2)-atan2d(a,b);
    t4 = asind(-C3/C4)-atan2d(a,b);
    
    theta3 = 180 + t4;
    theta4 = 180 + t3;
end