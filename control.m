function u = control(sigma,rho,x)
kappa = 10;
e = norm(x - sigma)/rho; % normalized error
eps = log((1+e)/(1-e)); % transformed error
u = -kappa * eps * (x - sigma);
end