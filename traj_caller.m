clear
clc
close all

x_start = 0;
y_start = 0;
x_end = 2;
y_end = 0.4;
init_yaw = 0;
final_yaw = pi/2.5;

x_offset = 0.4;
y_offset = 0;

viapoint_x = 0.5;
viapoint_y = 0;
viapoint_yaw = 0.0;

T_ph1 = 3;
T_ph2 = 4;

constrains = [x_start,y_start,x_end,y_end,init_yaw,final_yaw,x_offset,y_offset];
viapoint_profile = [viapoint_x,viapoint_y,viapoint_yaw];
T = [T_ph1,T_ph2];
pose_seq = generate_polynomial_trajectory_with_via_point(constrains,viapoint_profile,T);
figure(1)
plot(pose_seq(:,1),pose_seq(:,2),'LineWidth',4)

figure(2)
hold on
axis([0 max(pose_seq(:,1)) min(pose_seq(:,2)) max(max(pose_seq))])
% axis equal
steps = 100*(T_ph1+T_ph2);
for i=steps:-1:1
   figure(2)
   plot(pose_seq(i,1),pose_seq(i,2),'Marker','.','MarkerSize',5,'color','r');
   drawnow
end
 
data = load("E:\cpp_trajectory_planner\trajectory_generator\matrix_test.txt");
plot(data(:,1),data(:,2))
