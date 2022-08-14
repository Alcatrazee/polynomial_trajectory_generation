function path = gen_3rd_poly_rewrite(constrains,viapoints,T,steps_per_sec)

x_start = 0;
y_start = 0;
x_end = constrains(1);
y_end = constrains(2);

initial_direction = constrains(3);
end_direction = constrains(4);

viapoint_num = size(viapoints,1);

viapoints_with_head_tail = [[x_start,y_start,initial_direction];viapoints;[x_end,y_end,end_direction]];

% delta_T_vec = zeros(1,viapoint_num+1);
% v = 0.5; % can be modified
%
% for i=1:viapoint_num+1
%
% end

segaments = viapoint_num+1;
b_vec = zeros(segaments*4,1);

% positional constrains
position_constrains_mat = zeros(segaments*2,segaments*4);
for i=1:segaments
    position_constrains_mat((i-1)*2+1,(i-1)*4+1:(i-1)*4+4) = poly_derivative(0,0);
    position_constrains_mat((i-1)*2+2,(i-1)*4+1:(i-1)*4+4) = poly_derivative(0,viapoints_with_head_tail(i+1,1) - viapoints_with_head_tail(i,1));
    b_vec(2*i-1) = viapoints_with_head_tail(i,2);
    b_vec(2*i) = viapoints_with_head_tail(i+1,2);
end

velocity_constrains_mat = zeros(2,segaments*4);
velocity_constrains_mat(1,1:4) = poly_derivative(1,0);
velocity_constrains_mat(2,end-3:end) = poly_derivative(1,viapoints_with_head_tail(end,1) - viapoints_with_head_tail(end-1,1));
b_vec(2*segaments+1) = tan(initial_direction);
b_vec(2*segaments+2) = tan(end_direction);

continuity_constrains_mat = zeros(2*segaments-2,segaments*4);
for i = 1:(2*segaments-2)/2
        continuity_constrains_mat((i-1)*2+1,(i-1)*4+1:(i-1)*4+4) = poly_derivative(1,viapoints_with_head_tail(i+1,1) - viapoints_with_head_tail(i,1));
        continuity_constrains_mat((i-1)*2+1,i*4+1:i*4+4) = -poly_derivative(1,0);
        b_vec(2*segaments+2+(2*i-1)) = 0;
        continuity_constrains_mat((i-1)*2+2,(i-1)*4+1:(i-1)*4+4) = poly_derivative(2,viapoints_with_head_tail(i+1,1) - viapoints_with_head_tail(i,1));
        continuity_constrains_mat((i-1)*2+2,i*4+1:i*4+4) = -poly_derivative(2,0);
        b_vec(2*segaments+2+(2*i)) = 0;
end
% end continuity constrains
% continuity_constrains_mat(end-1,end-3:end) = poly_derivative(2,viapoints_with_head_tail(end,1) - viapoints_with_head_tail(end-1,1));
% continuity_constrains_mat(end,end-3:end) = -poly_derivative(2,0);


constrain_mat = [position_constrains_mat;velocity_constrains_mat;continuity_constrains_mat];
coffs = constrain_mat\b_vec;

x_series = zeros(segaments,1000);
for i=1:segaments
    x_series(i,1:end) = linspace(0,(viapoints_with_head_tail(i+1,1)-viapoints_with_head_tail(i,1)),1000);
end

y_series = zeros(segaments,1000);
for i=1:segaments
    for j=1:size(x_series,2)
       y_series(i,j) = polyval(flip(coffs((4*(i-1)+1):4*(i-1)+4)),x_series(i,j));
    end
end

subplot(2,1,1);
plot(x_series(1,:),y_series(1,:))
subplot(2,1,2);
plot(x_series(2,:),y_series(2,:))

x_all = zeros(1,size(x_series,2)*segaments);
y_all = zeros(1,size(y_series,2)*segaments);
for i=1:segaments
    if(i>1)
        x_all((i-1)*1000+1:(i-1)*1000+1000) = x_series(i,:) + x_all((i-1)*1000);
    else
        x_all((i-1)*1000+1:(i-1)*1000+1000) = x_series(i,:);
    end
    y_all((i-1)*size(x_series,2)+1:(i-1)*size(x_series,2)+size(x_series,2)) = y_series(i,:);
end

figure(2)
plot(x_all,y_all)

path = [x_all;y_all]';

end

function result = poly_derivative(order,T)
result = zeros(1,4);
for i=1:4
    n = i-1;
    s = order;
    if(n-s<0)
        result(i) = 0;
    else
        result(i) = factorial(n)/factorial(n-s) * T^(n-s);
    end
end
end