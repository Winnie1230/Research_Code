% time derivative of quaternion
function Q = Qq(q)
    q = q/norm(q);
    
    Q = 1/2*[-q(2) -q(3) -q(4) ;
              q(1)  q(4) -q(3) ; 
             -q(4)  q(1)  q(2) ; 
              q(3) -q(2)  q(1)];
end