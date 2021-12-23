% This function turns the quaternion into specific form to operate on
% angular velocities.

function e = Q(q)

    e = [-q(2) -q(3) -q(4);
          q(1)  q(4) -q(3);
         -q(4)  q(1)  q(2);
          q(3) -q(2)  q(1)] *0.5;

end