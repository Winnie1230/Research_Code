% Searching for the theta where torque_y is smaller than torque_x or
% torque_z
smaller = zeros(2,L);
Test_Wrench = F_E_g2;
for i = 1 : L
    smaller(1,i) = ( abs(Test_Wrench(4,i)) > abs(Test_Wrench(5,i)) );
    smaller(2,i) = ( abs(Test_Wrench(6,i)) > abs(Test_Wrench(5,i)) );
end