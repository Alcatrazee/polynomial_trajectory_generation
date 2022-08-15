function path = gen_3rd_poly_rewrite(viapoints,steps_per_sec)

viapoint_num = size(viapoints,1)-2;
segaments = viapoint_num+1;
coffs = generate_3rd_polynomial_coffs(viapoints);

y_series = zeros(1,steps_per_sec*viapoints(end,1));
k=1;
for i=1:segaments
    x_series = linspace(0,(viapoints(i+1,1)-viapoints(i,1)),(viapoints(i+1,1)-viapoints(i,1))*steps_per_sec);
    for j = 1:size(x_series,2)
        y_series(k) = polyval(flip(coffs((4*(i-1)+1):4*(i-1)+4)),x_series(j));
        k = k+1;
    end
end
x_series = linspace(viapoints(1,1),viapoints(end,1),(viapoints(end,1) - viapoints(1,1))*steps_per_sec);

% y_series = zeros(segaments,steps_per_sec);
% for i=1:segaments
%     for j=1:size(x_series,2)
%        y_series(i,j) = polyval(flip(coffs((4*(i-1)+1):4*(i-1)+4)),x_series(i,j));
%     end
% end

% x_all = linspace(viapoints(1,1),viapoints(end,1),steps_per_sec*viapoints(end,1));
% x_all = zeros(1,size(x_series,2)*segaments);
% y_all = zeros(1,size(x_series,2)*segaments);
% dir = zeros(1,size(x_series,2)*segaments); % TODO: add direvative 
% for i=1:segaments
%     if(i>1)
%         x_all((i-1)*steps_per_sec+1:(i-1)*steps_per_sec+steps_per_sec) = x_series(i,:) + x_all((i-1)*steps_per_sec);
%     else
%         x_all((i-1)*steps_per_sec+1:(i-1)*steps_per_sec+steps_per_sec) = x_series(i,:);
%     end
%     y_all((i-1)*size(x_series,2)+1:(i-1)*size(x_series,2)+size(x_series,2)) = y_series(i,:);
% end

% figure(2)
% plot(x_all,y_all)
% axis equal
% hold on
% for i = 1:viapoint_num
%    plot(x_all(i*steps_per_sec),y_all(i*steps_per_sec),'Marker','o')
% end


path = [x_series;y_series]';

end

