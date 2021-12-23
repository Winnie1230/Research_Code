%[A,B,C,D,E,F,G,l4] = calculate_coordinate(130,0.5*130,130,0.6*130,126,18,56,160);    %參考值
% 最小 17 deg 最大 160 deg

function Points = calculate_coordinate_flat(theta_deg)
    load('param.mat');
    %theta_deg = 30;
    %% 
    l2 = R-l1;
    l6 = l5l6 - l5;
    % upframe = up_deg/180;   %upframe是比例
    down_deg = 180-up_deg;
    downframe = down_deg/180;   %downframe是比例
    l8 = 2*R*sin(downframe*pi/2);
    theta0 = theta0_deg*pi/180;
    m = m_deg/180;  %m是比例
    theta = theta_deg*pi/180;

    %%
    l3 = 2*R*sin(m*pi/2);
    l7 = 2*R*sin((up_deg-theta0_deg-m_deg)*pi/180/2);
    arc_BF = (up_deg-theta0_deg)*pi/180; 
    BF = 2*R*sin(arc_BF/2);
    ang_BCF = acos((l3.^2+l7.^2-BF.^2)/(2*l3*l7)); 
    % Bx0 = R*sin(theta0);
    % By0 = R*cos(theta0);
    Cx0 = R*sin(theta0+m*pi);
    Cy0 = R*cos(theta0+m*pi);
    Ey0 = l1*cos(theta0)-sqrt(l1.^2*cos(theta0).^2-l1.^2+(l5+l6).^2);
    Dx0 = l1*l6*sin(theta0)/(l5+l6);
    Dy0 = l5*Ey0/(l5+l6) + l1*l6*cos(theta0)/(l5+l6);
    % Fx0 = R*sin(upframe*pi);
    % Fy0 = R*cos(upframe*pi);
    l4 = sqrt((Cx0-Dx0).^2+(Cy0-Dy0).^2); 

    Ax = l1*sin(theta);
    Ay = l1*cos(theta);
    Bx = R*sin(theta);
    By = R*cos(theta);
    Ex = 0;
    delta = (l1*cos(theta)).^2-(l1.^2-(l5+l6).^2);
    Ey = l1*cos(theta)-sqrt(delta);
    Dx = l1*l6*sin(theta)/(l5+l6);
    Dy = l5*Ey/(l5+l6) + l1*l6*cos(theta)/(l5+l6);

    % ang_OEA = atan2(Ax,(Ay-Ey));
    ang_OEA = acos((Ey.^2+l5l6.^2-l1.^2)/(2*(-Ey)*l5l6));
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
    
    % Export the output coordinates in 3D
    Points.A1 = [Ax,Ay]';
    Points.A2 = [-Ax,Ay]';
    Points.B1 = [Bx,By]';
    Points.B2 = [-Bx,By]';
    Points.C1 = [Cx,Cy]';
    Points.C2 = [-Cx,Cy]';
    Points.D1 = [Dx,Dy]';
    Points.D2 = [-Dx,Dy]';
    Points.E =  [Ex,Ey]';
    Points.F1 = [Fx,Fy]';
    Points.F2 = [-Fx,Fy]';
    Points.G =  [Gx,Gy]';
    
    Points.A1_3d = [Ax 0 Ay]';
    Points.A2_3d = [-Ax 0 Ay]';
    Points.B1_3d = [Bx 0 By]';
    Points.B2_3d = [-Bx 0 By]';
    Points.C1_3d = [Cx 0 Cy]';
    Points.C2_3d = [-Cx 0 Cy]';
    Points.D1_3d = [Dx 0 Dy]';
    Points.D2_3d = [-Dx 0 Dy]';
    Points.E_3d =  [Ex 0 Ey]';
    Points.F1_3d = [Fx 0 Fy]';
    Points.F2_3d = [-Fx 0 Fy]';
    Points.G_3d =  [Gx 0 Gy]';
end