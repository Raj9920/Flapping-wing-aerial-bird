% Parameters
height = 10;         % Height of the cylinder
radius = 2;          % Radius of the cylinder
num_turns = 5;       % Number of turns in the spiral
points_per_turn = 100; % Number of points per turn

% Generate the spiral trajectory
theta = linspace(0, num_turns * 2 * pi, num_turns * points_per_turn);
z = linspace(0, height, num_turns * points_per_turn);
x = radius * cos(theta);
y = radius * sin(theta);

% Create a figure
figure;
hold on;

% Plot the spiral trajectory
plot3(x, y, z, 'r', 'LineWidth', 2);

% Plot the rectangular cylinder
[X, Y, Z] = cylinder(radius);
Z = Z * height; % Scale the height
surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Cylinder surface

% Plot the bird object
bird_radius = 0.1; % Radius of the bird
bird_handle = plot3(x(1), y(1), z(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Initial position of the bird

% Set up the plot
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Spiral Trajectory with FWMAV Following the Path');
axis equal;
grid on;
view(3);

% Animation loop for the bird
for k = 1:length(x)
    % Update bird position
    set(bird_handle, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    pause(0.05); % Adjust speed of the bird
end

hold off;