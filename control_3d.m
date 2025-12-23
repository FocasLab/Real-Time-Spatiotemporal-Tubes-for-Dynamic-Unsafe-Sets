function u = control_3d(tube, rad, robot,velo,rho_L,rho_U,rho_d,rho_d_dot)
% u = 10*(tube-robot);
k=12;
a = 5;
e = norm(robot - tube)/rad;
ub =4;
%u = -ub*tanh(a*e).*(1-exp(-a*a*e.^2)) * (robot - tube) / norm(robot - tube);
v_d = real(-ub * (robot - tube) * log((1+e)/(1-e)));
for i=1:length(robot)

x=velo(i)-v_d(i);
e1=(x-0.5*(rho_U(i)+rho_L(i)))/(0.5*(rho_d(i)));
eps=log((1+e1)/(1-e1));
zeta=4/(rho_d(i)*(1-e1^2));
u(i)=-(k*zeta*eps-0.5*rho_d_dot(i)*e1);
end

u = real(u);
end
