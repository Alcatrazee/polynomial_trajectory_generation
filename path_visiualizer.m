close all
clear

final_yaw = 0;
initial_yaw = 0.0;  % should be always 0 due to the path is based on base link
x_start = 0;
initial_k = atan(initial_yaw);
x_end = 0.5; 
y_end = 0.4;
final_k = atan(final_yaw);
T_total = 5;

[xa0,xa1,xa2,xa3] = generate_3rd_trajectory(x_start,initial_k,y_end,final_k,x_end);
coff = [xa0,xa1,xa2,xa3];

T_curve = 3;
T_straight = T_total - T_curve;
steps_per_sec = 100;
steps = T_curve*steps_per_sec;

y_pose_curve = zeros(1,steps);
% x pose should be following a spline curve
x_pose = get_x_array(x_start,0.1,x_end,0,T_curve,steps_per_sec);
% x_pose = linspace(x_start,x_end,steps);
t = linspace(0,T_curve,steps);


for i=1:steps
    y_pose_curve(i) = polyval(flip(coff),x_pose(i));
end
figure(1)
subplot(3,1,1);
plot(x_pose,y_pose_curve,'linewidth',3)
xlabel('x pose')
ylabel('y pose')
subplot(3,1,2);
plot(t,x_pose,'linewidth',3)
xlabel('time')
ylabel('x pose')
subplot(3,1,3);
plot(t,y_pose_curve,'linewidth',3)
xlabel('time')
ylabel('y pose')
axis equal

figure(2)
hold on
axis([0 max(x_pose) min(y_pose_curve) max(y_pose_curve)])
axis equal
for i=steps:-1:1
   figure(2)
   plot(x_pose(i),y_pose_curve(i),'Marker','.','MarkerSize',5,'color','r');
   drawnow
%    pause(0.01)
end

function x_array = get_x_array(x_start,k_initial,x_end,k_final,T,steps_per_second)
    [xa0,xa1,xa2,xa3] = generate_3rd_trajectory(x_start,k_initial,x_end,0,T);
    coff = [xa0,xa1,xa2,xa3];
    step_amount = T*steps_per_second;
    t = linspace(0,T,step_amount);
    x_array = zeros(1,step_amount);
    for i=1:step_amount
       x_array(i) = polyval(flip(coff),t(i));
    end
    plot(t,x_array);
end