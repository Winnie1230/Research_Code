close all; clear all; clc;
% generate the animation of the transformation vertically(beta = 0)

%% Input parameters
R = 100; %mm
load(['param_R',num2str(R),'.mat']);

for row = 1 %1:length(H(:,1))
%     theta = theta0_deg : 5 : th_hmax;
%     beta = zeros(1,length(theta)); %30;
    theta = theta0_deg;
    beta = 0;
    O = [0,0];
    shift = [0;0];

    %% plot fig
    figure
    gcf_arr = [0.3,0.2,0.30,0.70];
    set(gcf,'unit','normalized','position',gcf_arr);
    axis_arr = [-2*R 2*R -4*R R];
    
    [l1,l2,l3,l4,l5,l6,l7,l8] = CalculateLink(R,l1,l5l6,l5,up_deg,theta0_deg,m_deg);
    
    for j = 1:length(theta)
        theta_deg = theta(j);
        
        beta_deg = beta(j);    
        clf();
    %     O = O_array2(:,j);
        [A,B,C,D,E,F,G] = CalculateCoordinate(R,l1,l2,l3,l4,l5,l6,l7,l8,theta_deg,up_deg,theta0_deg);
        
        [Oup, up] = upframe_coordinate(R,up_deg,B,C,F);         
        % Oup: center of the upper rim
        % up: the arc of the upper rim
        
        [Odn, dn] = downframe_coordinate(R,up_deg,F,G);
        % Odn: center of the upper rim
        % dn: the arc of the upper rim
        
        [yc] = masscenter_coordinate(up,dn); %calculate the centroid
        
        [AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,~,FL,~,~,upR,upL,~,OdnL,dnR,dnL,masscenter] = rotate_coordinate(O,A,B,C,D,E,F,Oup,up,Odn,dn,yc,beta_deg);    
        
        [O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter] = shift_coordinate(O,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter,shift);
        plot_linkleg2(O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter);
        
        axis equal;
%         axis(axis_arr);
        drawnow;
        frames(j) = getframe(gcf);	%use getframe to save every plot into frames
    end

    % name = strcat(folder,'fig\',num2str(H_row),' wheel');            
    % saveas(gcf,[name,'.fig']);
    % saveas(gcf,[name,'.jpg']);

    %% export plots to gif file
    % dt = 0.1; %set time interval of every plot(sec)
    % save every frame to move_pic.gif
    % for i=1:length(frames)
    %     [image,map]=frame2im(frames(i));
    %     [im,map2]=rgb2ind(image,128);
    %     if i==1
    %         imwrite(im,map2,[folder,'row',num2str(H_row),'.gif'],'gif','writeMode','overwrite','delaytime',dt,'loopcount',inf);
    %     else
    %         imwrite(im,map2,[folder,'row',num2str(H_row),'.gif'],'gif','writeMode','append','delaytime',dt);
    %     end
    % end
end