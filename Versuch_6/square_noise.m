num_square_points = 100;
num_outliers = num_square_points ;
total_points = num_square_points + num_outliers;

% Define the side length of the square
side_length = 10;

% Define the center of the dataset
center_x = 0;
center_y = 0;

% Generate points on the square with noise
noise_level = 0.2;
square_points = zeros(num_square_points, 2);
% Top side
square_points(1:num_square_points/4, :) = [linspace(-side_length/2, side_length/2, num_square_points/4)', (side_length/2) * ones(num_square_points/4, 1)];
% Right side
square_points(num_square_points/4+1:num_square_points/2, :) = [(side_length/2) * ones(num_square_points/4, 1), linspace(side_length/2, -side_length/2, num_square_points/4)'];
% Bottom side
square_points(num_square_points/2+1:3*num_square_points/4, :) = [linspace(side_length/2, -side_length/2, num_square_points/4)', (-side_length/2) * ones(num_square_points/4, 1)];
% Left side
square_points(3*num_square_points/4+1:end, :) = [(-side_length/2) * ones(num_square_points/4, 1), linspace(-side_length/2, side_length/2, num_square_points/4)'];

% Add noise to the square points
square_points = square_points + noise_level * randn(num_square_points, 2);

% Rotate the square by 45 degrees
theta = pi / 4; % 45 degrees in radians
rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
square_points = (rotation_matrix * square_points')';

% Generate random outlier points
outliers = side_length * 2 * (rand(num_outliers, 2) - 0.5);

% Combine square points and outliers
data = [square_points; outliers];

% Move the square to the center of the dataset
data(:,1) = data(:,1) + center_x;
data(:,2) = data(:,2) + center_y;

% Plot the points
figure;
scatter(data(:,1), data(:,2), 'filled');
title('Rotated Square Points with Noise and 50% Outliers');
xlabel('X');
ylabel('Y');
axis equal;