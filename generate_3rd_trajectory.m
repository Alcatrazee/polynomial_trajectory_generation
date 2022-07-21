function result = generate_3rd_trajectory(x_0,x_dot_0,x_t,x_dot_t,T)
    
    A_matrix = [1 0 0 0
                0 1 0 0
                1 T T^2 T^3
                0 1 2*T 3*T^2];
    b = [x_0,x_dot_0,x_t,x_dot_t]';
    
    result = A_matrix\b;
end