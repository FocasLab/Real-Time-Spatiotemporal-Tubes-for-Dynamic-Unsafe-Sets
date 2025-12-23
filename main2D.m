%% STT implementation for a single integrator system in a 2D environment
clc; 
clear; 
clf;

%% --- Parameters ---

% Environment parameters
S = [0.5, 1.5]; % Initial position
dS = 1; % Radius of initail set
eta = [7, 10];  % Target position
dT = 1; % Radius of target set
% Define dynamic obstacles (format: [ox, oy, rho_o, vel_o_x, vel_o_y])
obstacles = [4, 3, 0.5, -0.4, -0.2; 
             5, 6, 0.5, -0.4, -0.2; 
             6, 4, 0.5, 0.2, -0.4; 
             3, 6, 0.5, -0.2, 0.4;
             4.5, 9, 0.5, -0.2, 0];
%obstacles = createRandomObstacleMatrix(100); % Create 100 random obstacles
nObs = size(obstacles,1);

% STT parameters
k1 = 100; % coefficient k1
k2 = 100; % coefficient k2 for all j
k3 = 100; % coefficient k3 for all j
rho_max = 0.9; % Maximum allowable STT radius
rho_min = 0.1; % Minimum allowable STT radius
sigma = [1, 1]; % STT initial position sigma(0)

% Robot Parameters
x = S; % set initial robot position within initial set

% Simulation parameters
delt = 1e-2; % Step size for movement
tc = 16; % Prescribed time to reach goal
tf = 1.25*tc; % Simulation stop time
t = 1:delt:tf;

%% Plot setup
figure(1)
set(gcf, 'Position', [100, 100, 800*0.8, 1125*0.8]); 
set(gcf, 'Color', 'k');
hold on; grid on; box on;
axis([0 8 0 11.25]);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'FontSize', 18);
xlabel('X'); ylabel('Y');
title('Real-Time STT in 2D with Dynamic Obstacles');

% Plot Target
rectangle('Position',[eta(1)-dT eta(2)-dT 2*dT 2*dT],'Curvature',1,'FaceColor','g','EdgeColor','g','FaceAlpha',0.2)

% Plot obstacles
theta = linspace(0, 2*pi, 100);
obstacle_plots = gobjects(nObs, 1);
for o = 1:nObs
    ox = obstacles(o,1) + obstacles(o,3) * cos(theta);
    oy = obstacles(o,2) + obstacles(o,3) * sin(theta);
    obstacle_plots(o) = fill(ox, oy, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'r');
end

% Initialize robot trajectory plot
robot_plot = plot(x(1), x(2), 'ko', 'MarkerFaceColor', 'k');
STT_plot = plot(sigma(1), sigma(2), 'bo');
trajectory = plot(x(1), x(2), 'b-', 'LineWidth', 1.5);

%% STT loop
for iter = 1:length(t)
    % Update obstacle positions
    for o = 1:nObs
        obstacles(o,1) = obstacles(o,1) + delt * obstacles(o,4); % Update x position
        obstacles(o,2) = obstacles(o,2) + delt * obstacles(o,5); % Update y position
        ox = obstacles(o,1) + obstacles(o,3) * cos(theta);
        oy = obstacles(o,2) + obstacles(o,3) * sin(theta);
        set(obstacle_plots(o), 'XData', ox, 'YData', oy);
    end
    
    %% STT center dynamics (sigma-dot)
    % Compute first term of \dot{\sigma} in Equation (6)
    dsigma_1 = k1 * tc / (tc-t(iter)) * (eta-sigma);
    
    % Compute second term of \dot{\sigma} in Equation (6)
    dsigma_2 = [0, 0];
    rho = rho_max; % Initialize at default radius
    d = inf; 
    for o = 1:nObs
        obs_pos = obstacles(o,1:2);
        obs_radius = obstacles(o,3);
        denom = norm(sigma - obs_pos) - obs_radius - rho_min; % Distance to obstacle
        if denom <= rho_max
            % Compute m and v in the second term in Equation (6)
            m = (sigma - obs_pos) / denom^3; 
            N = null(m); v = N(:,1)';       
            dsigma_2 = dsigma_2 + k2*m + k3*v;
            
            % Compute smooth minimum distance to obstacles in Equation (9)
            d = smoothmin(d,  denom); 
        end
    end
    
    % Compute \dot(\sigma) in Equation (6)
    dsigma = dsigma_1 + dsigma_2;
    
    % Update STT center
    if norm(dsigma) > 0
        sigma = sigma + delt * (dsigma / norm(dsigma));
    end
    
    %% Compute STT radius \rho
    % Compute \rho as in Equation (15) ( solution of Equation (8) )
    if d < inf
        rho = smoothmin(rho_max, d);
    end
    
    %% Update Robot Position
    u = control(sigma,rho,x); % Compute control input as in Equation (18)
    x = x + delt * u;

    %% Update Plot
    set(robot_plot, 'XData', x(1), 'YData', x(2));
    set(STT_plot, 'XData', sigma(1), 'YData', sigma(2));
    set(trajectory, 'XData', [get(trajectory, 'XData'), x(1)], ...
                    'YData', [get(trajectory, 'YData'), x(2)]);
    delete(findobj('Type', 'line', 'Color', [0.3010 0.7450 0.9330])); 
    safety_x = sigma(1) + (rho - 0.1) * cos(theta);
    safety_y = sigma(2) + (rho - 0.1) * sin(theta);
    plot(safety_x, safety_y, 'color', [0.3010 0.7450 0.9330], 'LineStyle', '--', 'LineWidth', 1.5); 
    
    pause(delt/100); 
    
    if norm(sigma - eta) < 0.2
        disp(['Goal reached at t = ', num2str(t(iter))]);
        break;
    end
end