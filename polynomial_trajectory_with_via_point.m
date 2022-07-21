close all
clear

final_yaw = 0;
offset = 0.4;
initial_yaw = 0.0;  % should be always 0 due to the path is based on base link
x_start = 0;
via_point_dir = 0;
initial_k = atan(initial_yaw);
x_end = 0.5; 
y_end = 0.4;
final_k = atan(final_yaw);
x_via_point = 0.2;
k_via_point = atan(via_point_dir);
T_total = 5;

T_curve = 3;
T_straight = T_total - T_curve;

% phase 1 
coff1 = generate_3rd_trajectory(x_start,initial_k,x_start,initial_k,x_via_point);
% phase 2
coff2 = generate_3rd_trajectory(0,initial_k,y_end,final_k,x_end-x_via_point);

steps_per_sec = 100;
steps = T_total*steps_per_sec;

y_pose_curve = zeros(1,steps);
% x pose should be following a spline curve
x_pose = get_x_array(x_start,0,x_via_point,0.2,T_straight,steps_per_sec);
x_pose_phase_2 = get_x_array(x_via_point,0.2,x_end,0,T_curve,steps_per_sec);
x_pose = [x_pose , x_pose_phase_2(2:end),x_pose_phase_2(end) ];
% x_pose = linspace(x_start,x_end,steps);
t = linspace(0,T_total,steps);


for i=1:steps
    if(i<T_straight*steps_per_sec)
        y_pose_curve(i) = polyval(flip(coff1),x_pose(i));
    else
        y_pose_curve(i) = polyval(flip(coff2),x_pose(i)-x_pose(T_straight*steps_per_sec));
    end
end
x_pose  = x_pose + offset;

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
axis([0 max(x_pose) min(y_pose_curve) max([y_pose_curve x_pose])])
% axis equal
for i=steps:-1:1
   figure(2)
   plot(x_pose(i),y_pose_curve(i),'Marker','.','MarkerSize',5,'color','r');
   drawnow
end

function x_array = get_x_array(x_start,k_initial,x_end,k_final,T,steps_per_second)
    coff = generate_3rd_trajectory(x_start,k_initial,x_end,k_final,T);
    step_amount = T*steps_per_second;
    t = linspace(0,T,step_amount);
    x_array = zeros(1,step_amount);
    for i=1:step_amount
       x_array(i) = polyval(flip(coff),t(i));
    end
    plot(t,x_array);
end