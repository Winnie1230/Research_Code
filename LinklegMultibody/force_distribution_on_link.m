%% Analyze the results from LinkageWheel_force

% Link Alpha-1

a1O  = config(:,1,1);
a1A1 = config(:,2,1);
a1B1 = config(:,3,1);
a1 = plot3([a1O(1) a1A1(1) a1B1(1)] , [a1O(2) a1A1(2) a1B1(2)] , [a1O(3) a1A1(3) a1B1(3)] , 'Color' ,'black','LineWidth',2);
a = 0.7 ;
b = 0.8;
c = 1.3 ;
force_scale = 1e-8;
torque_scale = 1e-10;
hold on

for i = 1 : 100 : 1431
    arrow3d([ a1O(1)  a1O(1) + force_scale * F_O1_a1(1,i)],[ a1O(2)  a1O(2)  + force_scale * F_O1_a1(2,i)],[ a1O(3)  a1O(3) + force_scale * F_O1_a1(3,i)],a,b,c,'blue');
    arrow3d([a1A1(1) a1A1(1) + force_scale * F_A1_a1(1,i)],[a1A1(2)  a1A1(2) + force_scale * F_A1_a1(2,i)],[a1A1(3) a1A1(3) + force_scale * F_A1_a1(3,i)],a,b,c,'red');
    arrow3d([a1B1(1) a1B1(1) + force_scale * F_B1_a1(1,i)],[a1B1(2)  a1B1(2) + force_scale * F_B1_a1(2,i)],[a1B1(3) a1B1(3) + force_scale * F_B1_a1(3,i)],a,b,c,'green');

    arrow3d([ a1O(1)  a1O(1) + torque_scale * F_O1_a1(4,i)],[ a1O(2)  a1O(2)  + torque_scale * F_O1_a1(5,i)],[ a1O(3)  a1O(3) + torque_scale * F_O1_a1(6,i)],a,b,c,'#0072BD');
    arrow3d([a1A1(1) a1A1(1) + torque_scale * F_A1_a1(4,i)],[a1A1(2)  a1A1(2) + torque_scale * F_A1_a1(5,i)],[a1A1(3) a1A1(3) + torque_scale * F_A1_a1(6,i)],a,b,c,'#A2142F');
    arrow3d([a1B1(1) a1B1(1) + torque_scale * F_B1_a1(4,i)],[a1B1(2)  a1B1(2) + torque_scale * F_B1_a1(5,i)],[a1B1(3) a1B1(3) + torque_scale * F_B1_a1(6,i)],a,b,c,'#77AC30');

end

view(15,15);
set(gca,'DataAspectRatio',[1 1 1]);
axis([-100 100 -100 100 -100 100])
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');

hold off
