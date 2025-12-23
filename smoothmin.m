function m = smoothmin(a,b)
    nu = 100;
    m = -1/nu*log(exp(-nu*a)+exp(-nu*b));
    % m = min(a,b);
end