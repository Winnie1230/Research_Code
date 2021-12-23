function [Odn,dn] = downframe_coordinate(R,up_deg,F,G)
    down_deg = 180 - up_deg;
    Fx = F(1);  Fy = F(2);
    Gx = G(1);  Gy = G(2);
    % Odn_x1 = -(2*Fy*(Fy/2 + Gy/2 - (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 + (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2) - 2*Gy*(Fy/2 + Gy/2 - (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 + (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2) - Fx^2 - Fy^2 + Gx^2 + Gy^2)/(2*Fx - 2*Gx);
    % Odn_x2 = -(2*Fy*(Fy/2 + Gy/2 + (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 - (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2) - 2*Gy*(Fy/2 + Gy/2 + (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 - (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2) - Fx^2 - Fy^2 + Gx^2 + Gy^2)/(2*Fx - 2*Gx);
    % Odn_y1 = Fy/2 + Gy/2 - (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 + (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2;
    % Odn_y2 = Fy/2 + Gy/2 + (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 - (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2;
    % if Odn_x1 < Odn_x2 %Odn_x1^2 + Odn_y1^2 < Odn_x2^2 + Odn_y2^2
    %     Odn_x = Odn_x1;
    %     Odn_y = Odn_y1;
    % %     fprintf('x1,y1');
    % else
    %     Odn_x = Odn_x2;
    %     Odn_y = Odn_y2;
    % %     fprintf('x2,y2');
    % end
    Odn_x = -(2*Fy*(Fy/2 + Gy/2 + (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 - (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2) - 2*Gy*(Fy/2 + Gy/2 + (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 - (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2) - Fx^2 - Fy^2 + Gx^2 + Gy^2)/(2*Fx - 2*Gx);
    Odn_y = Fy/2 + Gy/2 + (Fx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2 - (Gx*(-(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2 - 4*R^2)/(Fx^2 - 2*Fx*Gx + Fy^2 - 2*Fy*Gy + Gx^2 + Gy^2))^(1/2))/2;
    Odn = [Odn_x, Odn_y];
    th_down_0 = asind((Fy-Odn_y)/R);  %deg
    theta_down = (th_down_0-down_deg):1:(th_down_0);     %一度一等分
    dn_x = Odn_x + R*cosd(theta_down);
    dn_y = Odn_y + R*sind(theta_down);
    dn = [dn_x; dn_y];
end