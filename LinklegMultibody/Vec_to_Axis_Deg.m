% Turning angular velocity into an axis of rotation and degree of rotation

function [v,theta] = Vec_to_Axis_Deg(vec)
    theta = norm(vec);
    if theta ~= 0 
       v = vec/theta;
    else
        v = [0 0 0]';
    end
end


