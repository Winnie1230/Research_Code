
figure();
hold on
scale = 2e-7;
% scale = 1e-5;
k = 1;
wrench.total_scaled = scale * wrench.total;
% Plot
a = 0.7 ;
b = 0.7;
c = 3 ;
% O
O_ = plot3([s(1,k) s(1,k) s(1,k)] , [s(2,k) s(2,k) s(2,k)] , [s(3,k) s(3,k) s(3,k)] , 'k.','MarkerSize',30);
% Ow = arrow3d([s(1,k) s(1,k) + wrench.total_scaled(1,k)],[s(2,k) s(2,k)+wrench.total_scaled(2,k)],[s(3,k) s(3,k)+wrench.total_scaled(3,k)],a,b,c,'black');

% alpha-1
a1_ = plot3([s(8,k) s(8,k) s(8,k)] , [s(9,k) s(9,k) s(9,k)] , [s(10,k) s(10,k) s(10,k)] ,'.', 'Color', '#0072BD','MarkerSize',20);
a1O  = s(8:10,k) + Rq(s(11:14,k))*config(:,1,1);
a1A1 = s(8:10,k) + Rq(s(11:14,k))*config(:,2,1);
a1B1 = s(8:10,k) + Rq(s(11:14,k))*config(:,3,1);
a1 = plot3([a1O(1) a1A1(1) a1B1(1)] , [a1O(2) a1A1(2) a1B1(2)] , [a1O(3) a1A1(3) a1B1(3)] , 'Color' ,'#0072BD','LineWidth',2);
% a1w = arrow3d([s(8,k) s(8,k) + wrench.total_scaled( 7,k)],[s(9,k) s(9,k)+wrench.total_scaled( 8,k)],[s(10,k) s(10,k) + wrench.total_scaled( 9,k)],a,b,c,'#0072BD');
% a1t = arrow3d([s(8,k) s(8,k) + wrench.total_scaled(10,k)],[s(9,k) s(9,k)+wrench.total_scaled(11,k)],[s(10,k) s(10,k) + wrench.total_scaled(12,k)],a,b,c,'#0072BD');

% alpha-2
a2_ = plot3([s(15,k) s(15,k) s(15,k)] , [s(16,k) s(16,k) s(16,k)] , [s(17,k) s(17,k) s(17,k)] ,'.', 'Color', '#0072BD','MarkerSize',20);
a2O  = s(15:17,k) + Rq(s(18:21,k))*config(:,1,2);
a2A2 = s(15:17,k) + Rq(s(18:21,k))*config(:,2,2);
a2B2 = s(15:17,k) + Rq(s(18:21,k))*config(:,3,2);
a2 = plot3([a2O(1) a2A2(1) a2B2(1)] , [a2O(2) a2A2(2) a2B2(2)] , [a2O(3) a2A2(3) a2B2(3)] , 'Color' ,'#0072BD','LineWidth',2);
% a2w = arrow3d([s(15,k) s(15,k) + wrench.total_scaled(13,k)],[s(16,k) s(16,k)+wrench.total_scaled(14,k)],[s(17,k) s(17,k) + wrench.total_scaled(15,k)],a,b,c,'#0072BD');
% a2t = arrow3d([s(15,k) s(15,k) + wrench.total_scaled(16,k)],[s(16,k) s(16,k)+wrench.total_scaled(17,k)],[s(17,k) s(17,k) + wrench.total_scaled(18,k)],a,b,c,'#0072BD');

% beta-1
b1_ = plot3([s(22,k) s(22,k) s(22,k)] , [s(23,k) s(23,k) s(23,k)] , [s(24,k) s(24,k) s(24,k)] ,'.', 'Color', '#D95319','MarkerSize',20);
b1B1 = s(22:24,k) + Rq(s(25:28,k))*config(:,1,3);
b1C1 = s(22:24,k) + Rq(s(25:28,k))*config(:,2,3);
b1F1 = s(22:24,k) + Rq(s(25:28,k))*config(:,3,3);
b1 = plot3([b1B1(1) b1C1(1) b1F1(1)] , [b1B1(2) b1C1(2) b1F1(2)] , [b1B1(3) b1C1(3) b1F1(3)] , 'Color' ,'#D95319','LineWidth',2);
% b1w = arrow3d([s(22,k) s(22,k) + wrench.total_scaled(19,k)],[s(23,k) s(23,k)+wrench.total_scaled(20,k)],[s(24,k) s(24,k) + wrench.total_scaled(21,k)],a,b,c,'#D95319');
% b1t = arrow3d([s(22,k) s(22,k) + wrench.total_scaled(22,k)],[s(23,k) s(23,k)+wrench.total_scaled(23,k)],[s(24,k) s(24,k) + wrench.total_scaled(24,k)],a,b,c,'#D95319');

% beta-2
b2_ = plot3([s(29,k) s(29,k) s(29,k)] , [s(30,k) s(30,k) s(30,k)] , [s(31,k) s(31,k) s(31,k)] ,'.', 'Color', '#D95319','MarkerSize',20);
b2B2 = s(29:31,k) + Rq(s(32:35,k))*config(:,1,4);
b2C2 = s(29:31,k) + Rq(s(32:35,k))*config(:,2,4);
b2F2 = s(29:31,k) + Rq(s(32:35,k))*config(:,3,4);
b2 = plot3([b2B2(1) b2C2(1) b2F2(1)] , [b2B2(2) b2C2(2) b2F2(2)] , [b2B2(3) b2C2(3) b2F2(3)] , 'Color' ,'#D95319','LineWidth',2);
% b2w = arrow3d([s(29,k) s(29,k) + wrench.total_scaled(25,k)],[s(30,k) s(30,k)+wrench.total_scaled(26,k)],[s(31,k) s(31,k) + wrench.total_scaled(27,k)],a,b,c,'#D95319');
% b2t = arrow3d([s(29,k) s(29,k) + wrench.total_scaled(28,k)],[s(30,k) s(30,k)+wrench.total_scaled(29,k)],[s(31,k) s(31,k) + wrench.total_scaled(30,k)],a,b,c,'#D95319');

% gamma-1
g1_ = plot3([s(36,k) s(36,k) s(36,k)] , [s(37,k) s(37,k) s(37,k)] , [s(38,k) s(38,k) s(38,k)] ,'.', 'Color', '#EDB120','MarkerSize',20);
g1A1 = s(36:38,k) + Rq(s(39:42,k))*config(:,1,5);
g1D1 = s(36:38,k) + Rq(s(39:42,k))*config(:,2,5);
g1E  = s(36:38,k) + Rq(s(39:42,k))*config(:,3,5);
g1 = plot3([g1A1(1) g1D1(1) g1E(1)] , [g1A1(2) g1D1(2) g1E(2)] , [g1A1(3) g1D1(3) g1E(3)] , 'Color' ,'#EDB120','LineWidth',2);
% g1w = arrow3d([s(36,k) s(36,k) + wrench.total_scaled(31,k)],[s(37,k) s(37,k)+wrench.total_scaled(32,k)],[s(38,k) s(38,k) + wrench.total_scaled(33,k)],a,b,c,'#EDB120' );
% g1t = arrow3d([s(36,k) s(36,k) + wrench.total_scaled(34,k)],[s(37,k) s(37,k)+wrench.total_scaled(35,k)],[s(38,k) s(38,k) + wrench.total_scaled(36,k)],a,b,c,'#EDB120' );

% gamma-2
g2_ = plot3([s(43,k) s(43,k) s(43,k)] , [s(44,k) s(44,k) s(44,k)] , [s(45,k) s(45,k) s(45,k)] ,'.', 'Color', '#EDB120','MarkerSize',20);
g2A2 = s(43:45,k) + Rq(s(46:49,k))*config(:,1,6);
g2D2 = s(43:45,k) + Rq(s(46:49,k))*config(:,2,6);
g2E  = s(43:45,k) + Rq(s(46:49,k))*config(:,3,6);
g2 = plot3([g2A2(1) g2D2(1) g2E(1)] , [g2A2(2) g2D2(2) g2E(2)] , [g2A2(3) g2D2(3) g2E(3)] , 'Color' ,'#EDB120','LineWidth',2);
% g2w = arrow3d([s(43,k) s(43,k) + wrench.total_scaled(37,k)],[s(44,k) s(44,k)+wrench.total_scaled(38,k)],[s(45,k) s(45,k) + wrench.total_scaled(39,k)],a,b,c,'#EDB120' );
% g2t = arrow3d([s(43,k) s(43,k) + wrench.total_scaled(40,k)],[s(44,k) s(44,k)+wrench.total_scaled(41,k)],[s(45,k) s(45,k) + wrench.total_scaled(42,k)],a,b,c,'#EDB120' );

% delta-1
d1_ = plot3([s(50,k) s(50,k) s(50,k)] , [s(51,k) s(51,k) s(51,k)] , [s(52,k) s(52,k) s(52,k)] ,'.', 'Color', '#77AC30','MarkerSize',20);
d1D1 = s(50:52,k) + Rq(s(53:56,k))*config(:,1,7);
d1C1 = s(50:52,k) + Rq(s(53:56,k))*config(:,2,7);
d1 = plot3([d1D1(1) d1C1(1)] , [d1D1(2) d1C1(2)] , [d1D1(3) d1C1(3)], 'Color' ,'#77AC30','LineWidth',2);
% d1w = arrow3d([s(50,k) s(50,k) + wrench.total_scaled(43,k)],[s(51,k) s(51,k)+wrench.total_scaled(44,k)],[s(52,k) s(52,k) + wrench.total_scaled(45,k)],a,b,c,'#77AC30' );
% d1t = arrow3d([s(50,k) s(50,k) + wrench.total_scaled(46,k)],[s(51,k) s(51,k)+wrench.total_scaled(47,k)],[s(52,k) s(52,k) + wrench.total_scaled(48,k)],a,b,c,'#77AC30' );

% delta-2
d2_ = plot3([s(57,k) s(57,k) s(57,k)] , [s(58,k) s(58,k) s(58,k)] , [s(59,k) s(59,k) s(59,k)] ,'.' , 'Color', '#77AC30','MarkerSize',20);
d2D2 = s(57:59,k) + Rq(s(60:63,k))*config(:,1,8);
d2C2 = s(57:59,k) + Rq(s(60:63,k))*config(:,2,8);
d2 = plot3([d2D2(1) d2C2(1)] , [d2D2(2) d2C2(2)] , [d2D2(3) d2C2(3)], 'Color' ,'#77AC30','LineWidth',2);
% d2w = arrow3d([s(57,k) s(57,k) + wrench.total_scaled(49,k)],[s(58,k) s(58,k)+wrench.total_scaled(50,k)],[s(59,k) s(59,k) + wrench.total_scaled(51,k)],a,b,c,'#77AC30' );
% d2t = arrow3d([s(57,k) s(57,k) + wrench.total_scaled(52,k)],[s(58,k) s(58,k)+wrench.total_scaled(53,k)],[s(59,k) s(59,k) + wrench.total_scaled(54,k)],a,b,c,'#77AC30' );

% epsilon-1
e1_ = plot3([s(64,k) s(64,k) s(64,k)] , [s(65,k) s(65,k) s(65,k)] , [s(66,k) s(66,k) s(66,k)] ,'.', 'Color', '#7E2F8E','MarkerSize',20);
e1F1 = s(64:66,k) + Rq(s(67:70,k))*config(:,1,9);
e1G  = s(64:66,k) + Rq(s(67:70,k))*config(:,2,9);
e1 = plot3([e1F1(1) e1G(1)] , [e1F1(2) e1G(2)] , [e1F1(3) e1G(3)], 'Color' , '#7E2F8E','LineWidth',2);
% e1w = arrow3d([s(64,k) s(64,k) + wrench.total_scaled(55,k)],[s(65,k) s(65,k)+wrench.total_scaled(56,k)],[s(66,k) s(66,k) + wrench.total_scaled(57,k)],a,b,c,'#7E2F8E' );
% e1t = arrow3d([s(64,k) s(64,k) + wrench.total_scaled(58,k)],[s(65,k) s(65,k)+wrench.total_scaled(59,k)],[s(66,k) s(66,k) + wrench.total_scaled(60,k)],a,b,c,'#7E2F8E' );

% epsilon-2
e2_ = plot3([s(71,k) s(71,k) s(71,k)] , [s(72,k) s(72,k) s(72,k)] , [s(73,k) s(73,k) s(73,k)] ,'.' , 'Color', '#7E2F8E','MarkerSize',20);
e2F2 = s(71:73,k) + Rq(s(74:77,k))*config(:,1,10);
e2G  = s(71:73,k) + Rq(s(74:77,k))*config(:,2,10);
e2 = plot3([e2F2(1) e2G(1)] , [e2F2(2) e2G(2)] , [e2F2(3) e2G(3)], 'Color' , '#7E2F8E','LineWidth',2);
% e2w = arrow3d([s(71,k) s(71,k) + wrench.total_scaled(61,k)],[s(72,k) s(72,k)+wrench.total_scaled(62,k)],[s(73,k) s(73,k) + wrench.total_scaled(63,k)],a,b,c,'#7E2F8E' );
% e2t = arrow3d([s(71,k) s(71,k) + wrench.total_scaled(64,k)],[s(72,k) s(72,k)+wrench.total_scaled(65,k)],[s(73,k) s(73,k) + wrench.total_scaled(66,k)],a,b,c,'#7E2F8E' );

% Reaction Forces
% F_O1_O_  = Rq(s( 4: 7)) *  F_O1_O(1:3,k);
% F_O2_O_  = Rq(s( 4: 7)) *  F_O2_O(1:3,k);
% F_O1_a1_ = Rq(s(11:14)) * F_O1_a1(1:3,k);
% F_A1_a1_ = Rq(s(11:14)) * F_A1_a1(1:3,k);
% F_B1_a1_ = Rq(s(11:14)) * F_B1_a1(1:3,k);
% F_O2_a2_ = Rq(s(18:21)) * F_O2_a2(1:3,k);
% F_A2_a2_ = Rq(s(18:21)) * F_A2_a2(1:3,k);
% F_B2_a2_ = Rq(s(18:21)) * F_B2_a2(1:3,k);
% F_B1_b1_ = Rq(s(25:28)) * F_B1_b1(1:3,k);
% F_C1_b1_ = Rq(s(25:28)) * F_C1_b1(1:3,k);
% F_F1_b1_ = Rq(s(25:28)) * F_F1_b1(1:3,k);
% F_B2_b2_ = Rq(s(32:35)) * F_B2_b2(1:3,k);
% F_C2_b2_ = Rq(s(32:35)) * F_C2_b2(1:3,k);
% F_F2_b2_ = Rq(s(32:35)) * F_F2_b2(1:3,k);
% F_A1_g1_ = Rq(s(39:42)) * F_A1_g1(1:3,k);
% F_D1_g1_ = Rq(s(39:42)) * F_D1_g1(1:3,k);
%  F_E_g1_ = Rq(s(39:42)) *  F_E_g1(1:3,k);
% F_A2_g2_ = Rq(s(46:49)) * F_A2_g2(1:3,k);
% F_D2_g2_ = Rq(s(46:49)) * F_D2_g2(1:3,k);
%  F_E_g2_ = Rq(s(46:49)) *  F_E_g2(1:3,k);
% F_D1_d1_ = Rq(s(53:56)) * F_D1_d1(1:3,k);
% F_C1_d1_ = Rq(s(53:56)) * F_C1_d1(1:3,k);
% F_D2_d2_ = Rq(s(60:63)) * F_D2_d2(1:3,k);
% F_C2_d2_ = Rq(s(60:63)) * F_C2_d2(1:3,k);
% F_F1_e1_ = Rq(s(67:70)) * F_F1_e1(1:3,k);
%  F_G_e1_ = Rq(s(67:70)) *  F_G_e1(1:3,k);
% F_F2_e2_ = Rq(s(74:77)) * F_F2_e2(1:3,k);
%  F_G_e2_ = Rq(s(74:77)) *  F_G_e2(1:3,k);
Fr_O_a1_ = Rq(s(11:14))  * Fr_O_a1(1:3,k);
Fr_O_a2_ = Rq(s(18:21))  * Fr_O_a2(1:3,k);
Fr_a1_O_ = Rq(s(4:7)) * Fr_a1_O(1:3,k);
Fr_a2_O_ = Rq(s(4:7)) * Fr_a2_O(1:3,k);

Tr_O_a1_ = Rq(s(11:14)) * Fr_O_a1(4:6,k);
Tr_O_a2_ = Rq(s(18:21)) * Fr_O_a2(4:6,k);
Tr_a1_O_ = Rq(s(4:7)) * Fr_a1_O(4:6,k);
Tr_a2_O_ = Rq(s(4:7)) * Fr_a2_O(4:6,k);

% 
% FA1a1 = arrow3d([a1A1(1) a1A1(1) + scale*F_A1_a1_(1)],[a1A1(2) a1A1(2) + scale*F_A1_a1_(2)],[a1A1(3) a1A1(3) + scale*F_A1_a1_(3)],a,b,c,'#0072BD' );
% FA1g1 = arrow3d([g1A1(1) g1A1(1) + scale*F_A1_g1_(1)],[g1A1(2) g1A1(2) + scale*F_A1_g1_(2)],[g1A1(3) g1A1(3) + scale*F_A1_g1_(3)],a,b,c,'#EDB120' );
% FA2a2 = arrow3d([a2A2(1) a2A2(1) + scale*F_A2_a2_(1)],[a2A2(2) a2A2(2) + scale*F_A2_a2_(2)],[a2A2(3) a2A2(3) + scale*F_A2_a2_(3)],a,b,c,'#0072BD' );
% FA2g2 = arrow3d([g2A2(1) g2A2(1) + scale*F_A2_g2_(1)],[g2A2(2) g2A2(2) + scale*F_A2_g2_(2)],[g2A2(3) g2A2(3) + scale*F_A2_g2_(3)],a,b,c,'#EDB120' );
% 
% FB1a1 = arrow3d([a1B1(1) a1B1(1) + scale*F_B1_a1_(1)],[a1B1(2) a1B1(2) + scale*F_B1_a1_(2)],[a1B1(3) a1B1(3) + scale*F_B1_a1_(3)],a,b,c,'#0072BD' );
% FB1b1 = arrow3d([b1B1(1) b1B1(1) + scale*F_B1_b1_(1)],[b1B1(2) b1B1(2) + scale*F_B1_b1_(2)],[b1B1(3) b1B1(3) + scale*F_B1_b1_(3)],a,b,c,'#D95319' );
% FB2a2 = arrow3d([a2B2(1) a2B2(1) + scale*F_B2_a2_(1)],[a2B2(2) a2B2(2) + scale*F_B2_a2_(2)],[a2B2(3) a2B2(3) + scale*F_B2_a2_(3)],a,b,c,'#0072BD' );
% FB2b2 = arrow3d([b2B2(1) b2B2(1) + scale*F_B2_b2_(1)],[b2B2(2) b2B2(2) + scale*F_B2_b2_(2)],[b2B2(3) b2B2(3) + scale*F_B2_b2_(3)],a,b,c,'#D95319' );
% 
% FC1d1 = arrow3d([d1C1(1) d1C1(1) + scale*F_C1_d1_(1)],[d1C1(2) d1C1(2) + scale*F_C1_d1_(2)],[d1C1(3) d1C1(3) + scale*F_C1_d1_(3)],a,b,c,'#77AC30' );
% FC1b1 = arrow3d([b1C1(1) b1C1(1) + scale*F_C1_b1_(1)],[b1C1(2) b1C1(2) + scale*F_C1_b1_(2)],[b1C1(3) b1C1(3) + scale*F_C1_b1_(3)],a,b,c,'#D95319' );
% FC2d2 = arrow3d([d2C2(1) d2C2(1) + scale*F_C2_d2_(1)],[d2C2(2) d2C2(2) + scale*F_C2_d2_(2)],[d2C2(3) d2C2(3) + scale*F_C2_d2_(3)],a,b,c,'#77AC30' );
% FC2b2 = arrow3d([b2C2(1) b2C2(1) + scale*F_C2_b2_(1)],[b2C2(2) b2C2(2) + scale*F_C2_b2_(2)],[b2C2(3) b2C2(3) + scale*F_C2_b2_(3)],a,b,c,'#D95319' );
% 
% FD1d1 = arrow3d([d1D1(1) d1D1(1) + scale*F_D1_d1_(1)],[d1D1(2) d1D1(2) + scale*F_D1_d1_(2)],[d1D1(3) d1D1(3) + scale*F_D1_d1_(3)],a,b,c,'#77AC30' );
% FD1g1 = arrow3d([g1D1(1) g1D1(1) + scale*F_D1_g1_(1)],[g1D1(2) g1D1(2) + scale*F_D1_g1_(2)],[g1D1(3) g1D1(3) + scale*F_D1_g1_(3)],a,b,c,'#EDB120' );
% FD2d2 = arrow3d([d2D2(1) d2D2(1) + scale*F_D2_d2_(1)],[d2D2(2) d2D2(2) + scale*F_D2_d2_(2)],[d2D2(3) d2D2(3) + scale*F_D2_d2_(3)],a,b,c,'#77AC30' );
% FD2g2 = arrow3d([g2D2(1) g2D2(1) + scale*F_D2_g2_(1)],[g2D2(2) g2D2(2) + scale*F_D2_g2_(2)],[g2D2(3) g2D2(3) + scale*F_D2_g2_(3)],a,b,c,'#EDB120' );
% 
% FEg1 = arrow3d([g1E(1) g1E(1) + scale*F_E_g1_(1)],[g1E(2) g1E(2) + scale*F_E_g1_(2)],[g1E(3) g1E(3) + scale*F_E_g1_(3)],a,b,c,'#EDB120' );
% FEg2 = arrow3d([g2E(1) g2E(1) + scale*F_E_g2_(1)],[g2E(2) g2E(2) + scale*F_E_g2_(2)],[g2E(3) g2E(3) + scale*F_E_g2_(3)],a,b,c,'#EDB120' );
% 
% FF1e1 = arrow3d([e1F1(1) e1F1(1) + scale*F_F1_e1_(1)],[e1F1(2) e1F1(2) + scale*F_F1_e1_(2)],[e1F1(3) e1F1(3) + scale*F_F1_e1_(3)],a,b,c,'#7E2F8E' );
% FF1b1 = arrow3d([b1F1(1) b1F1(1) + scale*F_F1_b1_(1)],[b1F1(2) b1F1(2) + scale*F_F1_b1_(2)],[b1F1(3) b1F1(3) + scale*F_F1_b1_(3)],a,b,c,'#D95319' );
% FF2e2 = arrow3d([e2F2(1) e2F2(1) + scale*F_F2_e2_(1)],[e2F2(2) e2F2(2) + scale*F_F2_e2_(2)],[e2F2(3) e2F2(3) + scale*F_F2_e2_(3)],a,b,c,'#7E2F8E' );
% FF2b2 = arrow3d([b2F2(1) b2F2(1) + scale*F_F2_b2_(1)],[b2F2(2) b2F2(2) + scale*F_F2_b2_(2)],[b2F2(3) b2F2(3) + scale*F_F2_b2_(3)],a,b,c,'#D95319' );
% 
% FGe1 = arrow3d([e1G(1) e1G(1) + scale*F_G_e1_(1)],[e1G(2) e1G(2) + scale*F_G_e1_(2)],[e1G(3) e1G(3) + scale*F_G_e1_(3)],a,b,c,'#7E2F8E' );
% FGe2 = arrow3d([e2G(1) e2G(1) + scale*F_G_e2_(1)],[e2G(2) e2G(2) + scale*F_G_e2_(2)],[e2G(3) e2G(3) + scale*F_G_e2_(3)],a,b,c,'#7E2F8E' );
% 
% FO1a1 = arrow3d([a1O(1) a1O(1) + scale*F_O1_a1_(1)],[a1O(2) a1O(2) + scale*F_O1_a1_(2)],[a1O(3) a1O(3) + scale*F_O1_a1_(3)],a,b,c,'#0072BD' );
% FO1O  = arrow3d([a1O(1) a1O(1) + scale*F_O1_O_(1)],[a1O(2) a1O(2) + scale*F_O1_O_(2)],[a1O(3) a1O(3) + scale*F_O1_O_(3)],a,b,c,'black' );
% FO2a2 = arrow3d([a2O(1) a2O(1) + scale*F_O2_a2_(1)],[a2O(2) a2O(2) + scale*F_O2_a2_(2)],[a2O(3) a2O(3) + scale*F_O2_a2_(3)],a,b,c,'#0072BD');
% FO2O  = arrow3d([a2O(1) a2O(1) + scale*F_O2_O_(1)],[a2O(2) a2O(2) + scale*F_O2_O_(2)],[a2O(3) a2O(3) + scale*F_O2_O_(3)],a,b,c,'black');
% 
% Gw = arrow3d([e2G(1) e2G(1) + scale*G_Force(1)],[e2G(2) e2G(2) + scale*G_Force(2)],[e2G(3) e2G(3) + scale*G_Force(3)],a,b,c,'red' );
% %REFw = arrow3d([125 125],[0 0],[-250 -250+scale*1e8],a,b,c,'red' );
% REFwy = arrow3d([125 125],[0 scale*1e8],[-250 -250],a,b,c,'red' );
% %REFt = text([115 115],[0 0],[-255 -255],"100 N");
% REFt = text([115 115],[0 0],[-255 -255],"100 N.m");

axis([-200 200 -200 200 s(3,k)-300 s(3,k)+100])

%view(15,15);
view(0,0);
set(gca,'DataAspectRatio',[1 1 1]);
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');

title("Linkage-Wheel Mechanism 400N " + string(theta) + " deg");

hold off
