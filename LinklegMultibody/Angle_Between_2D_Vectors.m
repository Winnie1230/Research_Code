% Finding the angle between two 2D vectors
% From v1 to v2 counter-clockwise

function theta = Angle_Between_2D_Vectors(v1,v2)

    n1 = norm(v1);
    n2 = norm(v2);
    
    if n1~=0 && n2~=0
        theta_c =  (v1'*v2/(n1*n2));
        theta_s =  (det([v1 v2])/(n1*n2));
        if theta_c == 1 && theta_s == 0
            theta = 0;
        elseif theta_c > 0 && theta_s > 0
            theta = acos(theta_c);
        elseif theta_c == 0 && theta_s == 1
            theta = pi/2;
        elseif theta_c < 0 && theta_s > 0
            theta = acos(theta_c);
        elseif theta_c == -1 && theta_s == 0
            theta = pi;
        elseif theta_c < 0 && theta_s < 0
            theta = 2*pi - acos(theta_c);
        elseif theta_c == 0 && theta_s == -1
            theta = 3*pi/2;
        elseif theta_c > 0 && theta_s < 0
            theta = 2*pi - acos(theta_c);
            
        end
    else
        theta = 0;
    end
end