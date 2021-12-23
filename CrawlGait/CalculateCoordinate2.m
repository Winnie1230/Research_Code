% Calculate coordinate of every point
function [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,theta_deg)
    load(LinkArcFile);
    theta = theta_deg*pi/180;

    Ax = l1*sin(theta);
    Ay = l1*cos(theta);
    Bx = R*sin(theta);
    By = R*cos(theta);
    Ex = 0;
    delta = (l1*cos(theta)).^2-(l1.^2-(l5+l6).^2);
    Ey = l1*cos(theta)-sqrt(delta);
    Dx = l1*l6*sin(theta)/(l5+l6);
    Dy = l5*Ey/(l5+l6) + l1*l6*cos(theta)/(l5+l6);

    BF = 2*R*sin(arc_BF/2);
    ang_BCF = acos((l3.^2+l7.^2-BF.^2)/(2*l3*l7)); 
    
    % ang_OEA = atan2(Ax,(Ay-Ey));
    ang_OEA = acos((Ey.^2+(l5+l6).^2-l1.^2)/(2*(-Ey)*(l5+l6)));
    ang_OAE = theta - ang_OEA;
    ang_DAB = pi - ang_OAE;
    BD = sqrt(l2.^2 + l5.^2 - 2*l2*l5*cos(ang_DAB));
    ang_BCD = acos((l3.^2+l4.^2-BD.^2)/(2*l3*l4));
    ang_ABD = acos((l2.^2+BD.^2-l5.^2)/(2*l2*BD));        
    ang_CBD = acos((l3.^2+BD.^2-l4.^2)/(2*l3*BD));
    phi = ang_CBD + ang_ABD - theta;
    ang_ADB = pi - ang_DAB - ang_ABD;
    ang_BDC = pi - ang_CBD - ang_BCD;
    ang_CDE = pi - ang_ADB - ang_BDC;

    Cx = R*sin(theta) + l3*sin(phi);
    Cy = R*cos(theta) - l3*cos(phi);

    % AC = sqrt(l2.^2+l3.^2-2*l2*l3*cos(theta+phi));
    % ang_DAC = acos((l5.^2+AC.^2-l4.^2)/(2*l5*AC));    %alpha
    % ang_DCA = acos((l4.^2+AC.^2-l5.^2)/(2*l4*AC)); %beta

    EC = sqrt(l6.^2+l4.^2-2*l4*l6*cos(ang_CDE));
    ang_DCE = acos((l4.^2+EC.^2-l6.^2)/(2*l4*EC));  %omega
    ang_DEC = pi - ang_CDE - ang_DCE;
    ang_ECF = ang_BCF - ang_BCD - ang_DCE;
    EF = sqrt(EC.^2+l7.^2-2*EC*l7*cos(ang_ECF));
    ang_CEF = acos((EC.^2+EF.^2-l7.^2)/(2*EC*EF));
    ang_FEG = pi-ang_OEA-ang_DEC-ang_CEF;   %sigma
    % % delta2 = l8.^2-(EF*sin(ang_FEG)).^2;
    % delta2 = EF*cos(ang_FEG).^2-(EF.^2-l8.^2);
    % EG = EF*cos(ang_FEG)+sqrt(delta2);

    Fx = EF*sin(ang_FEG);
    Fy = Ey - EF*cos(ang_FEG);
    EG = EF*cos(ang_FEG)+sqrt(l8.^2-(EF*sin(ang_FEG)).^2);
    Gx = 0;
    Gy = Ey - EG;

    A = [Ax,Ay]; B = [Bx,By]; C = [Cx,Cy]; D = [Dx,Dy]; E = [Ex,Ey]; F = [Fx,Fy]; G = [Gx,Gy];
end