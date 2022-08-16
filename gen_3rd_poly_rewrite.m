function path = gen_3rd_poly_rewrite(viapoints,T,steps_per_sec)

viapoint_num = size(viapoints,1)-2;
segaments = viapoint_num+1;

% get y(x)
coffs_y_x = generate_3rd_polynomial_coffs(viapoints);
y_series = zeros(1,steps_per_sec*viapoints(end,1))';

% get x(t)
x_position_viapoints = [T(:),viapoints(:,1),zeros(size(viapoints,1),1)];
coff_x = generate_3rd_polynomial_coffs(x_position_viapoints);
x_series = [];
x_series_len = zeros(segaments,1);
for i=1:segaments
   t = linspace(0,T(i+1)-T(i),steps_per_sec*(T(i+1)-T(i)));
   x_series = [x_series;polyval(flip(coff_x((i-1)*4+1:(i-1)*4+4)),t)];
   x_series_len(i) = steps_per_sec*(T(i+1)-T(i));
end


k=1;
for i=1:segaments
    for j = 1:x_series_len(i)
        if(i==1)
            y_series(k) = polyval(flip(coffs_y_x((4*(i-1)+1):4*(i-1)+4)),x_series(i,j));
        else
            y_series(k) = polyval(flip(coffs_y_x((4*(i-1)+1):4*(i-1)+4)),x_series(i,j) - x_series(i-1,x_series_len(i-1)));
        end
        k = k+1;
    end
end

real_series = [];
for i=1:segaments
%     if i==1
        real_series = [real_series x_series(i,1:x_series_len(i))];
%     else
%         real_series = [real_series x_series(i,1:x_series_len(i))+x_series(i-1,x_series_len(i-1))];
%     end
end





path = [real_series' y_series];

end

