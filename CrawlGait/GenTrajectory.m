function [theta_traj,beta_traj,phi_traj,actual_G_traj] = GenTrajectory(traj,Theta_Length,O,shift,LinkArcFile)
    % Input:
    % traj(Nx2 array): end_effector trajectory
    % OGmap_theta(.mat file): Use OG length to determine theta
    
    % Output:
    % theta_traj, beta_traj
    % phi(Nx2 array: column1->phi_L(motor bar L)
    %                column2->phi_R(motor bar R)
    % actual_G_traj: actual G point corresponds to theta and beta
    
    load(Theta_Length);
    
    % booking
    theta_traj = zeros(size(traj,1),1);
    beta_traj = zeros(size(traj,1),1);
    phi_traj = zeros(size(traj,1),2);
    actual_G_traj = zeros(size(traj,1),2);
    
    for i = 1:size(traj,1)
        des_G = traj(i,:)';
        OG = norm(des_G-(O+shift));
        [row,col] = find(leg_length < OG);
        theta_deg = OGmap_theta(max(row),1);
        theta_traj(i,1) = theta_deg;
        normal_OG = ((O+shift)-des_G)/OG;

        % determine beta direction
        u = [normal_OG;0]; %turn normal_OG to 3D coordinate
        v = [0;1;0];
        uvcross = cross(v,u);
        sgn = -sign(uvcross(3));
        
        CosTheta = max(min(dot(normal_OG,[0;1])/(norm(normal_OG)*norm([0;1])),1),-1);
        beta_deg = sgn*real(acosd(CosTheta));
        beta_traj(i,1) = beta_deg;
        phi = [1 1; -1 1]*[theta_deg;beta_deg];
        phi_traj(i,:) = phi';
        
        [A,B,C,D,E,F,G] = CalculateCoordinate2(LinkArcFile,theta_deg);
        
        % compute actual G
        actual_G = [cosd(beta_deg),sind(beta_deg);-sind(beta_deg),cosd(beta_deg)]*G';
        actual_G = actual_G + (O+shift);
        actual_G_traj(i,:) = actual_G';
    end
    
end