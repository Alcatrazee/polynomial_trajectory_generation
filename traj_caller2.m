clear
clc
close all

x_end = 2;
y_end = 0.4;
init_yaw = 0;
final_yaw = pi/4;

viapoint_x = 0.5;
viapoint_y = 0;
viapoint_yaw = 0.0;

T_ph1 = 3;
T_ph2 = 3;
steps_per_sec = 100;

constrains = [x_end,y_end,init_yaw,final_yaw];
viapoint_profile = [viapoint_x,viapoint_y,viapoint_yaw];
T = [T_ph1,T_ph2];

pose_seq = gen_3rd_poly_rewrite(constrains,viapoint_profile,T);
