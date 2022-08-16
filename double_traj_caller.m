clear
clc
close all

x_start = 0;
y_start = 0;
x_end = 1.8;
y_end = 1;
init_yaw = 0;
final_yaw = pi/4;

viapoint_x0 = 0;
viapoint_y0 = 0;
viapoint_vx0 = 0;
viapoint_vy0 = 0;

viapoint_x1 = 0.5;
viapoint_y1 = 0;
viapoint_vx1 = 0.0;
viapoint_vy1 = 0.0;

viapoint_x2 = 0.7;
viapoint_y2 = 0.4;
viapoint_vx2 = 0.3;
viapoint_vy2 = 0.0;

viapoint_x3 = 1;
viapoint_y3 = 1;
viapoint_vx3 = 0;
viapoint_vy3 = 0.0;

viapoint_x4 = 1.5;
viapoint_y4 = 1.5;
viapoint_vx4 = 0;
viapoint_vy4 = 0.0;

T_ph0 = 0;
T_ph1 = 0.5;
T_ph2 = 1;
T_ph3 = 1.5;
T_ph4 = 2;
steps_per_sec = 1000;

% x planning
viapoint_profile_x = [T_ph0,viapoint_x0,viapoint_vx0;
                    T_ph1,viapoint_x1,viapoint_vx1;
                    T_ph2,viapoint_x2,viapoint_vx2;
                    T_ph3,viapoint_x3,viapoint_vx3;
                    T_ph4,viapoint_x4,viapoint_vx4];
x_path = gen_3rd_poly_rewrite(viapoint_profile_x,steps_per_sec);
x_traj = x_path(:,2);

viapoint_profile_y = [T_ph0,viapoint_y0,viapoint_vy0;
                    T_ph1,viapoint_y1,viapoint_vy1;
                    T_ph2,viapoint_y2,viapoint_vy2;
                    T_ph3,viapoint_y3,viapoint_vy3;
                    T_ph4,viapoint_y4,viapoint_vy4];
y_path = gen_3rd_poly_rewrite(viapoint_profile_y,steps_per_sec);
y_traj = y_path(:,2);

time = x_path(:,1);
subplot(3,1,1)
plot(x_traj,y_traj)
subplot(3,1,2)
plot(time,x_traj)
subplot(3,1,3)
plot(time,y_traj)
