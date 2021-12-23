%plot_linkleg(130,0.5*130,130,0.6*130,126,18,56,160,0)

function plot_linkleg2(O,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter) %mm,deg
    x0 = O(1);
    y0 = O(2);
    %%
    hold on;
    plot(x0,y0,'ko','markersize',15,'LineWidth',2); % center
    plot([x0,BR(1)],[y0,BR(2)],'LineWidth',2);    %right motorlink
    plot([x0,BL(1)],[y0,BL(2)],'LineWidth',2); %left motorlink
    plot([AR(1),ER(1)],[AR(2),ER(2)],'LineWidth',2);   %right l56
    plot([AL(1),EL(1)],[AL(2),EL(2)],'LineWidth',2);   %left l56
    plot([DR(1),CR(1)],[DR(2),CR(2)],'LineWidth',2);   %right l4
    plot([DL(1),CL(1)],[DL(2),CL(2)],'LineWidth',2);   %left l4
    
    plot(upR(:,1),upR(:,2),'k','LineWidth',2);  %right upframe
    plot(upR(1,1),upR(1,2),'ko','markersize',4);
    
    plot(upL(:,1),upL(:,2),'k','LineWidth',2);  %left upframe
    plot(upL(1,1),upL(1,2),'ko','markersize',4);
    plot(dnR(:,1),dnR(:,2),'k','LineWidth',2);  %right downframe
    % plot(dnR(end,1),dnR(end,2),'ks','markersize',4);
    plot(dnL(:,1),dnL(:,2),'k','LineWidth',2);  %left downframe
%     plot(dnL(end,1),dnL(end,2),'ks','markersize',4);
    plot(masscenter(1),masscenter(2),'r+');     %masscenter

    % plot(N)
    axis equal;
    hold off;
end
