% This function constructs the friction force vector of the bearing
function Fr = friction_matrix(v)
    
    n = 11;

    H1  = [ zeros(3,3) -eye(3) ; zeros(3,3)  eye(3) ];
    H2 = [ zeros(3,3)  eye(3) ; zeros(3,3) -eye(3) ];
    X = zeros(6,6);

    B = ...
      [H1  H2   X   X   X   X   X   X   X   X   X ;
       H1   X  H2   X   X   X   X   X   X   X   X ;
        X  H1   X   X   X  H2   X   X   X   X   X ;
        X   X  H1   X   X   X  H2   X   X   X   X ;
        X  H1   X  H2   X   X   X   X   X   X   X ;
        X   X  H1   X  H2   X   X   X   X   X   X ;
        X   X   X  H1   X   X   X  H2   X   X   X ;
        X   X   X   X  H1   X   X   X  H2   X   X ;
        X   X   X   X   X  H1   X  H2   X   X   X ;
        X   X   X   X   X   X  H1   X  H2   X   X ;
        X   X   X   X   X  H1  H2   X   X   X   X ;
        X   X   X  H1   X   X   X   X   X  H2   X ;
        X   X   X   X  H1   X   X   X   X   X  H2 ;
        X   X   X   X   X   X   X   X   X  H1  H2 ; ];
    
    S = B*v;
    Fr = zeros(6*n,1);
    
    for i = 0 : n-1
        Fr(6*i+1:6*i+3) = unit(S(6*i+1:6*i+3));
        Fr(6*i+4:6*i+6) = unit(S(6*i+4:6*i+6));
    end
    
end