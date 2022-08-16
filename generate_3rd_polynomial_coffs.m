function coffs = generate_3rd_polynomial_coffs(viapoints)

% viapoints: all points including start point and end point

viapoint_num = size(viapoints,1) - 2;
segaments = viapoint_num+1;
b_vec = zeros(segaments*4,1);
% positional constrains
position_constrains_mat = zeros(segaments*2,segaments*4);
for i=1:segaments
    position_constrains_mat((i-1)*2+1,(i-1)*4+1:(i-1)*4+4) = poly_derivative(0,0);
    position_constrains_mat((i-1)*2+2,(i-1)*4+1:(i-1)*4+4) = poly_derivative(0,viapoints(i+1,1) - viapoints(i,1));
    b_vec(2*i-1) = viapoints(i,2);
    b_vec(2*i) = viapoints(i+1,2);
end

velocity_constrains_mat = zeros(2,segaments*4);
velocity_constrains_mat(1,1:4) = poly_derivative(1,0);
velocity_constrains_mat(2,end-3:end) = poly_derivative(1,viapoints(end,1) - viapoints(end-1,1));
b_vec(2*segaments+1) = tan(viapoints(1,3));
b_vec(2*segaments+2) = tan(viapoints(end,3));

continuity_constrains_mat = zeros(2*segaments-2,segaments*4);
for i = 1:(2*segaments-2)/2
    continuity_constrains_mat((i-1)*2+1,(i-1)*4+1:(i-1)*4+4) = poly_derivative(1,viapoints(i+1,1) - viapoints(i,1));
    continuity_constrains_mat((i-1)*2+1,i*4+1:i*4+4) = -poly_derivative(1,0);
    b_vec(2*segaments+2+(2*i-1)) = 0;
    continuity_constrains_mat((i-1)*2+2,(i-1)*4+1:(i-1)*4+4) = poly_derivative(2,viapoints(i+1,1) - viapoints(i,1));
    continuity_constrains_mat((i-1)*2+2,i*4+1:i*4+4) = -poly_derivative(2,0);
    b_vec(2*segaments+2+(2*i)) = 0;
end

constrain_mat = [position_constrains_mat;velocity_constrains_mat;continuity_constrains_mat];
coffs = constrain_mat\b_vec;

end