function [yc] = masscenter_coordinate(up,dn)
    % calculate the centroid (only consider y-dir because the graphic is symmetric)
    mass = 700; %(g)
    mi = mass/360;  % one division = 1deg
    up_c = (2*sum(up(2,:))-up(2,1)-up(2,end))/2;   %因為up_y有up_deg+1個值，乾脆全部取中間點當質點位置
    dn_c = (2*sum(dn(2,:))-dn(2,1)-dn(2,end))/2;   %因為dn_y有dn_deg+1個值，乾脆全部取中間點當質點位置
    yc = mi*(up_c+dn_c)*2/mass;  % times 2 because the graphic is symmetric
end