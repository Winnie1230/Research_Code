% Finding configurations and legal initial position/orientation conditions
% and relative positions between joints using Sharon senpai's forward kinematic algorithm

% Relative position of the center of masses need to be given by the user.

function [config,s0] = conditions_setup(theta,CoM,O_x,O_q)
    
    points = calculate_coordinate_flat(theta);
    % points: the point coordinate when O = [0;0] and beta=0
    
    %% Finding initial state ( s0 )
    % Linkage alpha-1
    alpha1_phi = Angle_Between_2D_Vectors([1 0]', points.A1 - [O_x(1) O_x(3)]');
    alpha1_q = skew(O_q)*[cos(alpha1_phi/2) 0 -sin(alpha1_phi/2) 0]'; %rotate along negative y-axis
    alpha1_x = O_x - Rq(alpha1_q)*CoM(:,1);
    
    % Linkage alpha-2
    alpha2_phi = Angle_Between_2D_Vectors([1 0]',points.A2 - [O_x(1) O_x(3)]');
    alpha2_q = skew(O_q)*[cos(alpha2_phi/2) 0 -sin(alpha2_phi/2) 0]';
    alpha2_x = O_x - Rq(alpha2_q)*CoM(:,2);

    s0 = [O_x; O_q;
          alpha1_x; alpha1_q; alpha2_x; alpha2_q];
    %% Finding relative joint positions ( configs )
    % transfer from global frame to local frame
    alpha1_A1 = Rq(  alpha1_q)'*(Rq(O_q)*points.A1_3d -   alpha1_x); % vector pointing from alpha-1 to A1 in alpha-1's frame
    alpha1_B1 = Rq(  alpha1_q)'*(Rq(O_q)*points.B1_3d -   alpha1_x);
    alpha2_A2 = Rq(  alpha2_q)'*(Rq(O_q)*points.A2_3d -   alpha2_x);
    alpha2_B2 = Rq(  alpha2_q)'*(Rq(O_q)*points.B2_3d -   alpha2_x);
    
%     plot3(O_x(1),O_x(2),O_x(3),'ko','MarkerSize',5,'MarkerFaceColor','k'); hold on
%     plot3(alpha1_x(1),alpha1_x(2),alpha1_x(3),'ro','MarkerSize',5,'MarkerFaceColor','r');
%     plot3(alpha2_x(1),alpha2_x(2),alpha2_x(3),'bo','MarkerSize',5,'MarkerFaceColor','b');
%     plot3(points.A1_3d(1),points.A1_3d(2),points.A1_3d(3),'ro','MarkerSize',5,'MarkerFaceColor','r');
%     plot3(points.B1_3d(1),points.B1_3d(2),points.B1_3d(3),'ro','MarkerSize',5,'MarkerFaceColor','r');
%     axis equal
%     axis([-100 100 -200 200 -100 100]);
%     view(0,0);

    config(:,:,1) = [CoM(:,1) alpha1_A1 alpha1_B1];
    config(:,:,2) = [CoM(:,2) alpha2_A2 alpha2_B2];
end