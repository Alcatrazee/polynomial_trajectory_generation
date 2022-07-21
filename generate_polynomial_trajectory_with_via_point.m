function pose_sequence = generate_polynomial_trajectory_with_via_point(constrains,viapoint_profile,T)
% this function is to create a 3rd ordered polynomial curve with 1
% viapoint, parameters:
% constrains:[x_start,y_start,x_end,y_end,initial_direction,final_direction,x_offset,y_offset]
% viapoint_profile:[x_pose,y_pose,via_point_dir,]
% T:[T_phase1,T_phase2]

x_start = constrains(1);
y_start = constrains(2);
x_end = constrains(3) - constrains(7);
y_end = constrains(4);
initial_direction = constrains(5);
final_direction = constrains(6);
x_offset = constrains(7);
y_offset = constrains(8);

via_point_x_pose = viapoint_profile(1) - x_offset;
via_point_y_pose = viapoint_profile(2) - y_offset;
via_point_dir = viapoint_profile(3);

T_phase1 = T(1);
T_phase2 = T(2);

% phase 1
coff1 = generate_3rd_trajectory(y_start,atan(initial_direction),via_point_y_pose,atan(via_point_dir),via_point_x_pose);
% phase 2
coff2 = generate_3rd_trajectory(0,atan(via_point_dir),y_end,atan(final_direction),x_end-via_point_x_pose);

steps_per_sec = 100;
steps = (T_phase1+T_phase2)*steps_per_sec;

y_pose = zeros(1,steps);
% x pose should be following a spline curve
% TODO: auto define value k
x_pose = get_x_array(x_start,0,via_point_x_pose,atan(0.05),T_phase1,steps_per_sec);
x_pose_phase_2 = get_x_array(via_point_x_pose,atan(0.05),x_end,0,T_phase2,steps_per_sec);
x_pose = [x_pose , x_pose_phase_2(2:end),x_pose_phase_2(end) ];
for i=1:steps
    if(i<T_phase1*steps_per_sec)
        y_pose(i) = polyval(flip(coff1),x_pose(i));
    else
        y_pose(i) = polyval(flip(coff2),x_pose(i)-x_pose(T_phase1*steps_per_sec));
    end
end
x_pose = x_pose + x_offset;
y_pose = y_pose + y_offset;
pose_sequence = [x_pose(:) y_pose(:)];



