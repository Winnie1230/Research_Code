function [Oup,up] = upframe_coordinate(R,up_deg,B,C,F)
    Bx = B(1); By = B(2);
    Cx = C(1); Cy = C(2);
    Fy = F(2);
    % [x1,y1] = solve('(x1-Bx)^2+(y1-By)^2=R^2','(x1-Cx)^2+(y1-Cy)^2=R^2');
    % Oup_x = subs(x1,[Bx,By,Cx,Cy],[Bx,By,Cx,Cy]);
    % Oup_x1 = Bx/2 + Cx/2 + (By*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2 - (Cy*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2;
    % Oup_x2 = Bx/2 + Cx/2 - (By*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2 + (Cy*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2; 
    % Oup_y1 = By/2 + Cy/2 - (Bx*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2 + (Cx*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2;
    % Oup_y2 = By/2 + Cy/2 + (Bx*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2 - (Cx*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2;
    % if Oup_x1 < Oup_x2 %Oup_x1^2 + Oup_y1^2 < Oup_x2^2 + Oup_y2^2
    %     Oup_x = Oup_x1;
    %     Oup_y = Oup_y1;
    % %     fprintf('x1,y1');
    % else
    %     Oup_x = Oup_x2;
    %     Oup_y = Oup_y2;
    % %     fprintf('x2,y2');
    % end
    Oup_x = Bx/2 + Cx/2 - (By*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2 + (Cy*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2; 
    Oup_y = By/2 + Cy/2 + (Bx*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2 - (Cx*(-(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2 - 4*R^2)/(Bx^2 - 2*Bx*Cx + By^2 - 2*By*Cy + Cx^2 + Cy^2))^(1/2))/2;
    Oup = [Oup_x, Oup_y];
    th_up_0 = asind((Fy - Oup_y)/R);  %deg
    theta_up = th_up_0 : 1 : (th_up_0+up_deg);   %one division = 1deg
    up_x = Oup_x + R*cosd(theta_up);
    up_y = Oup_y + R*sind(theta_up);
    up = [up_x; up_y];
end