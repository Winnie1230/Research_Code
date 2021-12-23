function theta = AngleBetween2Dvector(u,v)
    u = [u;0]; %turn normal_OG to 3D coordinate
    v = [v;0];
    uvcross = cross(v,u);
    sgn = sign(uvcross(3));
    CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
    theta = sgn*real(acosd(CosTheta));
end