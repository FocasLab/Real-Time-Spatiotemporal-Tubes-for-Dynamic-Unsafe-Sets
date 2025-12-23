%% STT implementation for a double integrator system in a 3D environment
clc;
clear;
clf;

%% Parameters

% Environment parameters
S = [1, 1, 1]; % Initial position
dS = 1; % Radius of initail set
eta = [8, 8, 8]; % Target position
dT = 1; % Radius of target set
% Define dynamic obstacles (format: [ox, oy, rho_o, vel_o_x, vel_o_y])
obstacles = [10, 10, 10, 0.5, -0.19, -0.19, -0.19;
             6,  6, 10, 0.5,  0.00,  0.00, -0.33;
             2.6,2.6,2.6,0.5, -0.60, -0.60, -0.60;
             5.2,5.2,0.0,1.4,  0.00,  0.00,  0.55];

nObs = size(obstacles,1);

% STT parameters
k1 = 300; % coefficient k1
k2 = 500; % coefficient k2 for all j
k3 = 1000; % coefficient k3 for all j
rho_max = 0.9; % Maximum allowable STT radius
rho_min = 0.1; % Minimum allowable STT radius
sigma = [1,1,1]; % STT initial position sigma(0)

% Robot Parameters
x = S; % set initial robot position within initial set
dx = [0 0 0]; % Robot velocity (second-order model)

% Simulation parameters
delt = 1e-2; % Step size for movement
tc = 27; % Prescribed time to reach goal
tf = 30; % Simulation stop time
t = 1:delt:tf;

%% Plot Setup
figure(1); 
set(gcf, 'Color', 'k');
hold on; grid on; box on;
axis([0 10 0 10 0 10]); axis square;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'FontSize', 18);
title('Real-Time STT in 3D with Dynamic Obstacles');

% Plot Target
[sx,sy,sz] = sphere(25);
surf(eta(1)+sx, eta(2)+sy, eta(3)+sz, ...
    'FaceAlpha',0.2,'EdgeColor','none','FaceColor','g');

% Plot obstacles
obstacle_plots = gobjects(nObs,1);
for o = 1:nObs
    surf_x = obstacles(o,1) + obstacles(o,4)*sx;
    surf_y = obstacles(o,2) + obstacles(o,4)*sy;
    surf_z = obstacles(o,3) + obstacles(o,4)*sz;
    obstacle_plots(o) = surf(surf_x,surf_y,surf_z, ...
        'FaceAlpha',0.5,'EdgeColor','none','FaceColor','r');
end

% Robot and STT plots
robot_plot = plot3(x(1),x(2),x(3),'ko','MarkerFaceColor','k');
STT_plot   = plot3(sigma(1),sigma(2),sigma(3),'bo','MarkerFaceColor','b');
traj_plot  = plot3(x(1),x(2),x(3),'b','LineWidth',1.5);

%% STT Loop
for iter = 1:length(t)
    % Update obstacle positions
    obstacles(:,1:3) = obstacles(:,1:3) + delt * obstacles(:,5:7);
    for o = 1:nObs
        set(obstacle_plots(o), ...
            'XData', obstacles(o,1) + obstacles(o,4)*sx, ...
            'YData', obstacles(o,2) + obstacles(o,4)*sy, ...
            'ZData', obstacles(o,3) + obstacles(o,4)*sz);
    end

    %% STT center dynamics (sigma-dot)
    % Compute first term of \dot{\sigma} in Equation (6)
    dsigma_1 = k1 * tc/(tc - t(iter)) * (eta - sigma);
    
    % Compute second term of \dot{\sigma} in Equation (6)
    dsigma_2 = [0 0 0];
    rho = rho_max; % Initialize at default radius
    d = inf;
    for o = 1:nObs
        obs_pos = obstacles(o,1:3);
        obs_radius = obstacles(o,4);
        denom = norm(sigma - obs_pos) - obs_radius - rho_min; % Distance to obstacle
        if denom <= rho_max
            % Compute m and v in the second term in Equation (6)
            m = (sigma - obs_pos) / denom^3;
            N = null(m); v = N(:,1)'; 
            dsigma_2 = dsigma_2 + k2*m + k3*v;
            
            % Compute smooth minimum distance to obstacles in Equation (9)
            d = smoothmin(d,denom);
        end
    end
    
    % Compute \dot(\sigma) in Equation (6)
    dsigma = dsigma_1 + dsigma_2;
    
    % Update STT center
    if norm(dsigma) > 0
        sigma = sigma + delt * dsigma / norm(dsigma);
    end
    
    %% Compute STT radius \rho
    % Compute \rho as in Equation (15) ( solution of Equation (8) )
    if d < inf
        rho = smoothmin(rho_max, d);
    end

    %% Update Robot Position

    % Funnel bounds for second-order system
    gamma_L = -4*exp(-0.1*t(iter))*ones(1,3);
    gamma_U =  4*exp(-0.1*t(iter))*ones(1,3);
    gamma_d = gamma_U - gamma_L;
    gamma_d_dot = -0.8*exp(-0.1*t(iter))*ones(1,3);
    
    % Compute control input as in Equation (18)
    u = control_3d(sigma,rho,x,dx,gamma_L,gamma_U,gamma_d,gamma_d_dot); 

    dx = dx + delt * u;
    x = x + delt * dx;

    %% Update Plot
    set(robot_plot,'XData',x(1),'YData',x(2),'ZData',x(3));
    set(STT_plot,'XData',sigma(1),'YData',sigma(2),'ZData',sigma(3));
    set(traj_plot, 'XData',[traj_plot.XData x(1)], ...
                   'YData',[traj_plot.YData x(2)], ...
                   'ZData',[traj_plot.ZData x(3)]);
    delete(findobj(gca,'Tag','safe_sphere'));
    surf(sigma(1)+(rho-0.05)*sx, ...
         sigma(2)+(rho-0.05)*sy, ...
         sigma(3)+(rho-0.05)*sz, ...
         'FaceAlpha',0.2,'EdgeColor','none', ...
         'FaceColor',[0.3 0.75 0.93],'Tag','safe_sphere');

    drawnow;
    [az, el] = view;
    view(az + 120/tc*0.02, el);
    pause(delt/10);

    if norm(sigma - eta) < 0.2
        fprintf('Target reached at t = %.2f s\n',t(iter));
        break;
    end
end
