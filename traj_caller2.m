clear
clc
close all

x_start = 0;
y_start = 0;
x_end = 1.8;
y_end = 1;
init_yaw = 0;
final_yaw = pi/4;

viapoint_x = 0.5;
viapoint_y = 0;
viapoint_yaw = 0.0;

viapoint_x2 = 1;
viapoint_y2 = 0.4;
viapoint_yaw2 = 0.3;

viapoint_x3 = 1.5;
viapoint_y3 = 0.5;
viapoint_yaw3 = 0.3;

T_ph1 = 3;
T_ph2 = 3;
T_ph3 = 3;
steps_per_sec = 100;

viapoint_profile = [x_start,y_start,init_yaw;
                    viapoint_x,viapoint_y,viapoint_yaw;
                    viapoint_x2,viapoint_y2,viapoint_yaw2;
                    viapoint_x3,viapoint_y3,viapoint_yaw3;
                    x_end,y_end,final_yaw];
T = [T_ph1,T_ph2];

path = gen_3rd_poly_rewrite(viapoint_profile,T,100);


x_traj = path(:,1);
y_traj = path(:,2);
figure(2)
plot(y_traj,y_traj)
axis equal
hold on
for i = 1:3
   plot(x_traj(i*steps_per_sec),y_traj(i*steps_per_sec),'Marker','o')
end
