function PlotLinkleg(ParamFile,A,B,C,D,E,F,G,O,shift,beta_deg)
    load(ParamFile);
    
    % plot linkleg
    [Oup, up] = upframe_coordinate(R,up_deg,B,C,F);
    [Odn, dn] = downframe_coordinate(R,up_deg,F,G);
    [yc] = masscenter_coordinate(up,dn); %calculate the centroid

    [AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,~,FL,~,~,upR,upL,~,OdnL,dnR,dnL,masscenter] = rotate_coordinate(O,A,B,C,D,E,F,Oup,up,Odn,dn,yc,beta_deg);
    [O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter] = shift_coordinate(O,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter,shift);
    plot_linkleg2(O_real,AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,upR,upL,dnR,dnL,masscenter); hold on
end