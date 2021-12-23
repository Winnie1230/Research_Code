function PlotQuiver(p1,dp,linewidth,c,headsize,linespec)
%     dp = p2 - p1
    quiver(p1(1),p1(2),dp(1),dp(2),0,'LineWidth',linewidth,'Color',c,'MaxHeadSize',headsize,'LineStyle',linespec);
end