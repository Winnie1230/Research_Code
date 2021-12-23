% Find the maximum force/torque of each joint 
mmgs2mkgs = [1e-6 1e-6 1e-6 1e-9 1e-9 1e-9]';
F_O1_O_max  = mmgs2mkgs .* max(abs(F_O1_O),[],2);
F_O1_a1_max = mmgs2mkgs .* max(abs(F_O1_a1),[],2);
F_O2_O_max  = mmgs2mkgs .* max(abs(F_O2_O),[],2);
F_O2_a2_max = mmgs2mkgs .* max(abs(F_O2_a2),[],2);
F_A1_g1_max = mmgs2mkgs .* max(abs(F_A1_g1),[],2);
F_A1_a1_max = mmgs2mkgs .* max(abs(F_A1_a1),[],2);
F_A2_g2_max = mmgs2mkgs .* max(abs(F_A2_g2),[],2);
F_A2_a2_max = mmgs2mkgs .* max(abs(F_A2_a2),[],2);
F_B1_a1_max = mmgs2mkgs .* max(abs(F_B1_a1),[],2);
F_B1_b1_max = mmgs2mkgs .* max(abs(F_B1_b1),[],2);
F_B2_a2_max = mmgs2mkgs .* max(abs(F_B2_a2),[],2);
F_B2_b2_max = mmgs2mkgs .* max(abs(F_B2_b2),[],2);
F_C1_d1_max = mmgs2mkgs .* max(abs(F_C1_d1),[],2);
F_C1_b1_max = mmgs2mkgs .* max(abs(F_C1_b1),[],2);
F_C2_d2_max = mmgs2mkgs .* max(abs(F_C2_d2),[],2);
F_C2_b2_max = mmgs2mkgs .* max(abs(F_C2_b2),[],2);
F_D1_g1_max = mmgs2mkgs .* max(abs(F_D1_g1),[],2);
F_D1_d1_max = mmgs2mkgs .* max(abs(F_D1_d1),[],2);
F_D2_g2_max = mmgs2mkgs .* max(abs(F_D2_g2),[],2);
F_D2_d2_max = mmgs2mkgs .* max(abs(F_D2_d2),[],2);
F_E_g2_max  = mmgs2mkgs .* max(abs(F_E_g2),[],2);
F_E_g1_max  = mmgs2mkgs .* max(abs(F_E_g1),[],2);
F_F1_b1_max = mmgs2mkgs .* max(abs(F_F1_b1),[],2);
F_F1_e1_max = mmgs2mkgs .* max(abs(F_F1_e1),[],2);
F_F2_b2_max = mmgs2mkgs .* max(abs(F_F2_b2),[],2);
F_F2_e2_max = mmgs2mkgs .* max(abs(F_F2_e2),[],2);
F_G_e2_max  = mmgs2mkgs .* max(abs(F_G_e2),[],2);
F_G_e1_max  = mmgs2mkgs .* max(abs(F_G_e1),[],2);
Wrenches = [{'Force_x(N)'} {'Force_y(N)'} {'Force_z(N)'} {'Torque_x(N.m)'} {'Torque_y(N.m)'} {'Torque_z(N.m)'}]';

Max = table(Wrenches,F_O1_O_max,F_O1_a1_max,F_O2_O_max,F_O2_a2_max,F_A1_g1_max,F_A1_a1_max,F_A2_g2_max,F_A2_a2_max,F_B1_a1_max, ...
F_B1_b1_max,F_B2_a2_max,F_B2_b2_max,F_C1_d1_max,F_C1_b1_max,F_C2_d2_max,F_C2_b2_max,F_D1_g1_max,F_D1_d1_max,F_D2_g2_max, ...
F_D2_d2_max,F_E_g2_max,F_E_g1_max,F_F1_b1_max,F_F1_e1_max,F_F2_b2_max,F_F2_e2_max,F_G_e2_max,F_G_e1_max);

% save("joint_maximum_wrench.mat", ...
% 'F_O1_O_max','F_O1_a1_max','F_O2_O_max','F_O2_a2_max' ,'F_A1_g1_max' ,'F_A1_a1_max','F_A2_g2_max','F_A2_a2_max', ...
% 'F_B1_a1_max','F_B1_b1_max','F_B2_a2_max','F_B2_b2_max','F_C1_d1_max','F_C1_b1_max','F_C2_d2_max','F_C2_b2_max', ...
% 'F_D1_g1_max','F_D1_d1_max','F_D2_g2_max','F_D2_d2_max','F_E_g2_max','F_E_g1_max','F_F1_b1_max','F_F1_e1_max', ...
% 'F_F2_b2_max','F_F2_e2_max','F_G_e2_max','F_G_e1_max');

writetable(Max,'Joint_Max_Wrench.xlsx','Sheet','Default','WriteVariableNames',true);



