% The operation of cross product and quaternion multiplication can be linearized 
% by matricize the former vector/quaternion in the operation into a skew symmetric matrix.

% This function turns the vector/quaternion into "the" skew symmetric matrix

function Q = skew(q)
    if length(q) == 3
        Q = [    0  -q(3)   q(2);
              q(3)      0  -q(1);
             -q(2)   q(1)     0];
    else
        Q = [q(1) -q(2)  -q(3)  -q(4)
             q(2)  q(1)  -q(4)   q(3);
             q(3)  q(4)   q(1)  -q(2);
             q(4) -q(3)   q(2)   q(1)];
    end
end