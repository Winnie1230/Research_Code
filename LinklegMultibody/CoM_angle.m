function com_angle = CoM_angle(CoM)
    O_x = [0 0 0]';

    % Linkage alpha-1
    alpha1_phi = Angle_Between_2D_Vectors([1 0]', [CoM(1,1) CoM(3,1)]' - [O_x(1) O_x(3)]');

    % Linkage alpha-2
    alpha2_phi = Angle_Between_2D_Vectors([1 0]', [CoM(1,2) CoM(3,2)]' - [O_x(1) O_x(3)]');

    com_angle.alpha1_phi = alpha1_phi;
    com_angle.alpha2_phi = alpha2_phi;
end