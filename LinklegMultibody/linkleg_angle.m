% Finding angle between linkage and positive x-axis
% points: the point coordinate when O=[0;0] and beta=0

function angle = linkleg_angle(points)
    O_x = [0 0 0]';

    %% Finding initial state ( s0 )
    % Linkage alpha-1
    alpha1_phi = Angle_Between_2D_Vectors([1 0]', points.A1 - [O_x(1) O_x(3)]');

    % Linkage alpha-2
    alpha2_phi = Angle_Between_2D_Vectors([1 0]',points.A2 - [O_x(1) O_x(3)]');

    % Linkage gamma-1
    gamma1_phi = Angle_Between_2D_Vectors([1 0]',points.E - points.A1);

    % Linkage gamma-2
    gamma2_phi = Angle_Between_2D_Vectors([1 0]',points.E - points.A2);

    % Linkage beta-1
    beta1_phi = Angle_Between_2D_Vectors([1 0]', points.F1 - points.B1);

    % Linkage beta-2
    beta2_phi = Angle_Between_2D_Vectors([1 0]', points.F2 - points.B2);

    % Linkage delta-1
    delta1_phi = Angle_Between_2D_Vectors([1 0]', points.C1 - points.D1);

    % Linkage delta-2
    delta2_phi = Angle_Between_2D_Vectors([1 0]', points.C2 - points.D2);

    % Linkage epsilon-1
    epsilon1_phi = Angle_Between_2D_Vectors([1 0]', points.G - points.F1);

    % Linkage epsilon-2
    epsilon2_phi = Angle_Between_2D_Vectors([1 0]', points.G - points.F2);

    angle.alpha1_phi = alpha1_phi;
    angle.alpha2_phi = alpha2_phi;
    angle.gamma1_phi = gamma1_phi;
    angle.gamma2_phi = gamma2_phi;
    angle.beta1_phi = beta1_phi;
    angle.beta2_phi = beta2_phi;
    angle.delta1_phi = delta1_phi;
    angle.delta2_phi = delta2_phi;
    angle.epsilon1_phi = epsilon1_phi;
    angle.epsilon2_phi = epsilon2_phi;
end