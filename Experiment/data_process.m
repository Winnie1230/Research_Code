clear all; close all; clc;

csv = load('C:\Users\BioRoLaT470S\Documents\r09522826\Experiment\Linkleg_48V\1215\1215_test23.csv');

%% Remove data
% remove it if all columns are zero

% mask = (data(:,1) == 0);
data = csv(find(csv(:,1) ~= 0),:);

threshold = 0.04;
mod_pos1 = mod(data(:,13),2*pi);
mod_pos1(mod_pos1(:,1) <= threshold) = 2*pi-mod_pos1(mod_pos1(:,1) <= threshold);
mod_pos2 = mod(data(:,14),2*pi);
mod_pos2(mod_pos2(:,1) <= threshold) = 2*pi-mod_pos2(mod_pos2(:,1) <= threshold);

subplot(3,1,1);
plot(data(:,1),data(:,3),'Color','#0072BD','DisplayName','position cmd','LineWidth',2); hold on
% plot(data(:,1),data(:,5),'Color','#D95319','LineStyle','--','DisplayName','encoder reply','LineWidth',2);
plot(data(:,1),mod_pos1(:,1),'Color','#D95319','LineStyle','--','DisplayName','encoder reply','LineWidth',2);
% plot(data(:,1),mod_pos2(:,1),'Color','#D95319','LineStyle','--','DisplayName','encoder reply','LineWidth',2);

% title({['No load wheel mode'],['Forward speed: 1 rad/s, Recover speed: 5 rad/s'],['Motor1 position']},'Interpreter','latex');
% title({['24V violent jump no contact'],['Recover speed: 100 deg/s'],['Motor1(R) position']},'Interpreter','latex');
% title({['24V violent jump contact'],['Recover speed: 100 deg/s'],['Motor1(R) position']},'Interpreter','latex');
% title({['24V slow motion contact'],['Forward speed: 100 deg/s, Recover speed: 100 deg/s'],['Motor1(R) position']},'Interpreter','latex');
% title({['48V slow motion'],['Forward speed: 100 deg/s, Recover speed: 100 deg/s'],['Motor1(R) position']},'Interpreter','latex');
% title({['48V'],['Motor1(R) position']},'Interpreter','latex');
title({['24V'],['Motor1(R) position']},'Interpreter','latex');

xlabel('t[s]','Interpreter','latex');
ylabel('pos[rad]','Interpreter','latex');
legend('Location','best');
axis([min(data(:,1)) max(data(:,1)) -0.1 2*pi+0.2])

subplot(3,1,2);
plot(data(:,1), data(:,8),'Color','#7E2F8E','DisplayName','position cmd','LineWidth',2); hold on
% plot(data(:,1),data(:,10),'Color','#EDB120','LineStyle','--','DisplayName','encoder reply','LineWidth',2);
plot(data(:,1),mod_pos2(:,1),'Color','#EDB120','LineStyle','--','DisplayName','encoder reply','LineWidth',2);
% plot(data(:,1),mod_pos1(:,1),'Color','#EDB120','LineStyle','--','DisplayName','encoder reply','LineWidth',2);

title('Motor2(L) position','Interpreter','latex');
xlabel('t[sec]','Interpreter','latex');
ylabel('pos[rad]','Interpreter','latex');
legend('Location','best');
axis([min(data(:,1)) max(data(:,1)) -0.1 2*pi+0.2])

currentR_max = max(abs(data(:,7)));
currentL_max = max(abs(data(:,12)));

subplot(3,1,3);
plot(data(:,1),data(:,7),'Color','#0072BD','DisplayName','current1(R)','LineWidth',2); hold on
plot(data(:,1),data(:,12),'Color','#D95319','DisplayName','current2(L)','LineWidth',2);
% plot(data(:,1),data(:,12),'Color','#0072BD','DisplayName','current1(R)','LineWidth',2); hold on
% plot(data(:,1),data(:,7),'Color','#D95319','DisplayName','current2(L)','LineWidth',2);
title('Motor current','Interpreter','latex');
xlabel('t[sec]','Interpreter','latex');
ylabel('current[A]','Interpreter','latex');
legend('Location','best');
axis([min(data(:,1)) max(data(:,1)) -15 10]);

posR_err = sqrt(mean((data(:,3)-data(:,5)).^2));
posR_var = std(abs(data(:,3)-data(:,5)));
posL_err = sqrt(mean((data(:,8)-data(:,10)).^2));
posL_var = std(abs(data(:,8)-data(:,10)));



% figure();
% subplot(2,1,1);
% plot(data(:,1),data(:,15),'DisplayName','CAN TX2RX TIME1');
% legend
% subplot(2,1,2);
% plot(data(:,1),data(:,16),'DisplayName','CAN TX2RX TIME2');
% legend
