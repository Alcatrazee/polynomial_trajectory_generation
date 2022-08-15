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