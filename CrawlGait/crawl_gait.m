clear all; close all; clc;

r = 50; %mm
T = 8; %sec
dt = 0.1; %sec
swing = 8/4; %swing phase time
stance = 8/4*3; % stance phase time

swing_ang = linspace(0,pi,swing/dt+1)';
swing_traj = [r*cos(swing_ang(1:end-1)), r*sin(swing_ang(1:end-1))];
stance_dx = linspace(-r,r,stance/dt+1)';
stance_traj = [stance_dx(1:end-1),zeros(length(stance_dx)-1,1)];
traj_rf = [swing_traj;stance_traj];
traj_lf = [stance_traj;swing_traj];
traj_lh = [stance_traj(3/4*(T/dt)*2/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*2/3,:)];
traj_rh = [stance_traj(3/4*(T/dt)*1/3+1:end,:);swing_traj;stance_traj(1:3/4*(T/dt)*1/3,:)];

figure;

plot(traj_rf(:,1),traj_rf(:,2),'k-'); hold on
axis([-r r 0 r]);

h1 = animatedline('Color','b','Marker','o','MarkerFaceColor','b','LineWidth',1); %RF
h2 = animatedline('Color','r','Marker','o','MarkerFaceColor','r','LineWidth',1); %LF
h3 = animatedline('Color','g','Marker','o','MarkerFaceColor','g','LineWidth',1); %LF
h4 = animatedline('Color','y','Marker','o','MarkerFaceColor','y','LineWidth',1); %LF

del = 0.01; % time between animation frames
for run = 1:3
    for i = 1:T/dt
        addpoints(h1,traj_rf(i,1),traj_rf(i,2));
        addpoints(h2,traj_lf(i,1),traj_lf(i,2));
        addpoints(h3,traj_rh(i,1),traj_rh(i,2));
        addpoints(h4,traj_lh(i,1),traj_lh(i,2));

        drawnow;
        % ----- save to gif -----
        frame = getframe(1);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        filename = ['crawl_gait.gif'];
        if i == 1
            imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',del);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',del);
        end
        % -----------------------
        
        clearpoints(h1);
        clearpoints(h2);
        clearpoints(h3);
        clearpoints(h4);
    end
end



