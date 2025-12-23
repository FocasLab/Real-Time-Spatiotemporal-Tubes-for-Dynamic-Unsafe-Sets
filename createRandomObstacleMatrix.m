function obs = createRandomObstacleMatrix(n)
    % Create a random matrix with n rows and 5 columns
    % Column 1-2: values between 1 and 8: Initial Position of obstacle
    % Column 3: values between 0.1 and 0.4: Radius of Obstacle
    % Column 4-5: values between -1 and 1: Velocity of Obstacle

    col1_2 = 1 + (8 - 1) * rand(n, 2);        % Uniform [1, 8]
    col3 = 0.1 + (0.4 - 0.1) * rand(n, 1);    % Uniform [0.1, 0.4]
    col4_5 = -1 + 2 * rand(n, 2);             % Uniform [-1, 1]

    obs = [col1_2, col3, col4_5];
end