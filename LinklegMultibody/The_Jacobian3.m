function J = The_Jacobian3(s,ns_,config)
     
     % Rotate ns and config to the world frame;
     n =  11;
     ns = zeros(3,3,n);
     ns(:,:,n) = Rq(s(4:7))*ns_(:,:,n);
     for i = 1:n-1
            ns(:,:,i) = Rq(s( 7*i+4 : 7*i+7 ))*   ns_(:,:,i);
        config(:,:,i) = Rq(s( 7*i+4 : 7*i+7 ))*config(:,:,i);
     end
     
     % Making small jacobians first and then construct a bigger one with all the constraints
     
     % O1 O/a1
     J_O1_O         = [  eye(3)            zeros(3,3) ; zeros(1,3)   ns(:, 1,11)' ; zeros(1,3)   ns(: ,3,11)' ];
     J_O1_a1        = [ -eye(3)  skew(config(:,1, 1)) ; zeros(1,3)  -ns(:, 1,11)' ; zeros(1,3)  -ns(:, 3,11)' ];   
     
     % O2 O/a2
     J_O2_O         = [  eye(3)            zeros(3,3) ; zeros(1,3)   ns(:, 1,11)' ; zeros(1,3)   ns(:, 3,11)' ];
     J_O2_a2        = [ -eye(3)  skew(config(:,1, 2)) ; zeros(1,3)  -ns(:, 1,11)' ; zeros(1,3)  -ns(:, 3,11)' ];   
     
     % A1 g1/a1
     J_A1_g1        = [  eye(3) -skew(config(:,1, 5)) ; zeros(1,3)   ns(:, 1, 5)' ; zeros(1,3)   ns(:, 3, 5)' ];
     J_A1_a1        = [ -eye(3)  skew(config(:,2, 1)) ; zeros(1,3)  -ns(:, 1, 5)' ; zeros(1,3)  -ns(:, 3, 5)' ];   
     
     % A2 g2/a2
     J_A2_g2        = [  eye(3) -skew(config(:,1, 6)) ; zeros(1,3)   ns(:, 1, 6)' ; zeros(1,3)   ns(:, 3, 6)' ];
     J_A2_a2        = [ -eye(3)  skew(config(:,2, 2)) ; zeros(1,3)  -ns(:, 1, 6)' ; zeros(1,3)  -ns(:, 3, 6)' ];   
     
     % B1 a1/b1
     J_B1_a1        = [  eye(3) -skew(config(:,3, 1)) ; zeros(1,3)   ns(:, 1, 1)' ; zeros(1,3)   ns(:, 3, 1)' ];
     J_B1_b1        = [ -eye(3)  skew(config(:,1, 3)) ; zeros(1,3)  -ns(:, 1, 1)' ; zeros(1,3)  -ns(:, 3, 1)' ];   
     
     % B2 a2/b2
     J_B2_a2        = [  eye(3) -skew(config(:,3, 2)) ; zeros(1,3)   ns(:, 1, 2)' ; zeros(1,3)   ns(:, 3, 2)' ];
     J_B2_b2        = [ -eye(3)  skew(config(:,1, 4)) ; zeros(1,3)  -ns(:, 1, 2)' ; zeros(1,3)  -ns(:, 3, 2)' ];   
     
     % C1 d1/b1
     J_C1_d1        = [  eye(3) -skew(config(:,2, 7)) ; zeros(1,3)   ns(:, 1, 7)' ; zeros(1,3)   ns(:, 3, 7)' ];
     J_C1_b1        = [ -eye(3)  skew(config(:,2, 3)) ; zeros(1,3)  -ns(:, 1, 7)' ; zeros(1,3)  -ns(:, 3, 7)' ];   
     
     % C2 d2/b2
     J_C2_d2        = [  eye(3) -skew(config(:,2, 8)) ; zeros(1,3)   ns(:, 1, 8)' ; zeros(1,3)   ns(:, 3, 8)' ];
     J_C2_b2        = [ -eye(3)  skew(config(:,2, 4)) ; zeros(1,3)  -ns(:, 1, 8)' ; zeros(1,3)  -ns(:, 3, 8)' ];   
     
     % D1 g1/d1
     J_D1_g1        = [  eye(3) -skew(config(:,2, 5)) ; zeros(1,3)   ns(:, 1, 5)' ; zeros(1,3)   ns(:, 3, 5)' ];
     J_D1_d1        = [ -eye(3)  skew(config(:,1, 7)) ; zeros(1,3)  -ns(:, 1, 5)' ; zeros(1,3)  -ns(:, 3, 5)' ];   
     
     % D2 g2/d2
     J_D2_g2        = [  eye(3) -skew(config(:,2, 6)) ; zeros(1,3)   ns(:, 1, 6)' ; zeros(1,3)   ns(:, 3, 6)' ];
     J_D2_d2        = [ -eye(3)  skew(config(:,1, 8)) ; zeros(1,3)  -ns(:, 1, 6)' ; zeros(1,3)  -ns(:, 3, 6)' ];   
     
     % E  g2/g1
     J_E_g2         = [  eye(3) -skew(config(:,3, 6)) ; zeros(1,3)   ns(:, 1, 6)' ; zeros(1,3)   ns(:, 3, 6)' ];
     J_E_g1         = [ -eye(3)  skew(config(:,3, 5)) ; zeros(1,3)  -ns(:, 1, 6)' ; zeros(1,3)  -ns(:, 3, 6)' ];   
     
     % F1 b1/e1
     J_F1_b1        = [  eye(3) -skew(config(:,3, 3)) ; zeros(1,3)   ns(:, 1, 3)' ; zeros(1,3)   ns(:, 3, 3)' ];
     J_F1_e1        = [ -eye(3)  skew(config(:,1, 9)) ; zeros(1,3)  -ns(:, 1, 3)' ; zeros(1,3)  -ns(:, 3, 3)' ];   
    
     % F2 b2/e2
     J_F2_b2        = [  eye(3) -skew(config(:,3, 4)) ; zeros(1,3)   ns(:, 1, 4)' ; zeros(1,3)   ns(:, 3, 4)' ];
     J_F2_e2        = [ -eye(3)  skew(config(:,1,10)) ; zeros(1,3)  -ns(:, 1, 4)' ; zeros(1,3)  -ns(:, 3, 4)' ];   
     
     % G  e2/e1
     J_G_e2         = [  eye(3) -skew(config(:,2,10)) ; zeros(1,3)   ns(:, 1,10)' ; zeros(1,3)   ns(:, 3,10)' ];
     J_G_e1         = [ -eye(3)  skew(config(:,2, 9)) ; zeros(1,3)  -ns(:, 1,10)' ; zeros(1,3)  -ns(:, 3,10)' ];   
     
     % Jr_O O/alpha1
     Jr_O = [zeros(1,3)   ns(:,2,11)']; %1x6

     % Jr_a1 O/alpha1
     Jr_a1 = [zeros(1,3) -ns(:,2,1)' ]; %1x6

     % Jr_a2 O/alpha2
     Jr_a2 = [zeros(1,3) -ns(:,2,2)' ]; %1x6

     % Start building the big Jacobian
     
     X = zeros(5,6);
     x = zeros(1,6);
     
     %          O        a1        a2        b1        b2        g1        g2        d1        d2        e1        e2    
     J = [ J_O1_O   J_O1_a1         X         X         X         X         X         X         X         X         X;    % O1
           J_O2_O         X   J_O2_a2         X         X         X         X         X         X         X         X;    % O2
                X   J_A1_a1         X         X         X   J_A1_g1         X         X         X         X         X;    % A1
                X         X   J_A2_a2         X         X         X   J_A2_g2         X         X         X         X;    % A2
                X   J_B1_a1         X   J_B1_b1         X         X         X         X         X         X         X;    % B1
                X         X   J_B2_a2         X   J_B2_b2         X         X         X         X         X         X;    % B2
                X         X         X   J_C1_b1         X         X         X   J_C1_d1         X         X         X;    % C1
                X         X         X         X   J_C2_b2         X         X         X   J_C2_d2         X         X;    % C2
                X         X         X         X         X   J_D1_g1         X   J_D1_d1         X         X         X;    % D1
                X         X         X         X         X         X   J_D2_g2         X   J_D2_d2         X         X;    % D2
                X         X         X         X         X    J_E_g1    J_E_g2         X         X         X         X;    % E
                X         X         X   J_F1_b1         X         X         X         X         X   J_F1_e1         X;    % F1
                X         X         X         X   J_F2_b2         X         X         X         X         X   J_F2_e2;    % F2
                X         X         X         X         X         X         X         X         X    J_G_e1    J_G_e2;    % G
             Jr_O     Jr_a1         x         x         x         x         x         x         x         x         x;    % restrict rotation O/alpha1
             Jr_O         x     Jr_a2         x         x         x         x         x         x         x         x ];  % restrict rotation O/alpha2
end