function [AR,AL,BR,BL,CR,CL,DR,DL,ER,EL,FR,FL,OupR,OupL,upR,upL,OdnR,OdnL,dnR,dnL,masscenter] = rotate_coordinate(O,A,B,C,D,E,F,Oup,up,Odn,dn,yc,beta_deg)
    %% 
    rotation = [cosd(beta_deg),sind(beta_deg);-sind(beta_deg),cosd(beta_deg)];
    
    AR = rotation*[A(1);A(2)]+[O(1);O(2)];
    AL = rotation*[-A(1);A(2)]+[O(1);O(2)];
    BR = rotation*[B(1);B(2)]+[O(1);O(2)];
    BL = rotation*[-B(1);B(2)]+[O(1);O(2)];
    CR = rotation*[C(1);C(2)]+[O(1);O(2)];
    CL = rotation*[-C(1);C(2)]+[O(1);O(2)];
    DR = rotation*[D(1);D(2)]+[O(1);O(2)];
    DL = rotation*[-D(1);D(2)]+[O(1);O(2)];
    ER = rotation*[E(1);E(2)]+[O(1);O(2)];
    EL = rotation*[-E(1);E(2)]+[O(1);O(2)];
    FR = rotation*[F(1);F(2)]+[O(1);O(2)];
    FL = rotation*[-F(1);F(2)]+[O(1);O(2)];
    % GR = rotation*[G(1);G(2)]+[O(1);O(2)];
    % GL = rotation*[-G(1);G(2)]+[O(1);O(2)];
    OupR = rotation*[Oup(1);Oup(2)]+[O(1);O(2)];
    OupL = rotation*[-Oup(1);Oup(2)]+[O(1);O(2)];
    
    upR = (rotation*[up(1,:);up(2,:)]+[O(1);O(2)])';
    upL = (rotation*[-up(1,:);up(2,:)]+[O(1);O(2)])';
    
    OdnR = rotation*[Odn(1);Odn(2)]+[O(1);O(2)];
    OdnL = rotation*[-Odn(1);Odn(2)]+[O(1);O(2)];
    
%     dnR = rotation*[dn(:,1);dn(:,2)]+[O(1);O(2)];
%     dnL = rotation*[-dn(:,1);dn(:,2)]+[O(1);O(2)];
    dnR = (rotation*[dn(1,:);dn(2,:)]+[O(1);O(2)])';
    dnL = (rotation*[-dn(1,:);dn(2,:)]+[O(1);O(2)])';

    masscenter = rotation*[0;yc]+[O(1);O(2)];
end