function x_array = get_x_array(x_start,k_initial,x_end,k_final,T,steps_per_second)
    coff = generate_3rd_trajectory(x_start,k_initial,x_end,k_final,T);
    step_amount = T*steps_per_second;
    t = linspace(0,T,step_amount);
    x_array = zeros(1,step_amount);
    for i=1:step_amount
       x_array(i) = polyval(flip(coff),t(i));
    end
end