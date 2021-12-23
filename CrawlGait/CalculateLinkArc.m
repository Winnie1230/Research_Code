% last change: 2020/09/23
% Calculate length of every link by loading parameters file and export LinkArc_Rxxx.mat
% input: ParamFile(path,filename)
% output: LinkArc_RXXX.mat filename
function filename = CalculateLinkArc(ParamFile)
    load(ParamFile);
    l2 = R-l1; % length of l2
    l6 = l5l6 - l5; % length of l6
    
    upframe = up_deg/180;   % upframe(ratio) = nHF
    down_deg = 180 - up_deg; % lower rim(deg)
    downframe = down_deg/180;   %downframe(ratio)
    
    l8 = 2*R*sin(downframe*pi/2); %length of l8
    
    theta0 = theta0_deg*pi/180;
    m = m_deg/180;  %m(ratio) = nBC

    l3 = 2*R*sin(m*pi/2); % length of l3
    l7 = 2*R*sin((up_deg-theta0_deg-m_deg)*pi/180/2); % length of l7
    
    % ----- the initial coordinate of point C, D, E -----
    Cx0 = R*sin(theta0+m*pi);
    Cy0 = R*cos(theta0+m*pi);
    Ey0 = l1*cos(theta0)-sqrt(l1.^2*cos(theta0).^2-l1.^2+(l5+l6).^2);
    Dx0 = l1*l6*sin(theta0)/(l5+l6);
    Dy0 = l5*Ey0/(l5+l6) + l1*l6*cos(theta0)/(l5+l6);
    % Fx0 = R*sin(upframe*pi);
    % Fy0 = R*cos(upframe*pi);
    % ----------------------------------------------------
   
    l4 = sqrt((Cx0-Dx0).^2+(Cy0-Dy0).^2); % length of l4
    
    % ----- calculate arc_BF -----
    arc_BF = (up_deg-theta0_deg)*pi/180;
    % ----------------------------
    
    filename = ['LinkArc_R',num2str(R),'.mat'];
    save(filename,'R','l1','l2','l3','l4','l5','l6','l7','l8','arc_BF');
end