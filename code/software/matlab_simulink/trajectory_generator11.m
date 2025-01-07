%% Position trajectory generation
function [X_ref, X_dot_ref, X_dot_dot_ref, Y_ref, Y_dot_ref, Y_dot_dot_ref, Z_ref, Z_dot_ref, Z_dot_dot_ref, psi_ref] = trajectory_generator(t, r, f, height_i, height_f)

    % Initialize constants
    constants = initial_constants();
    Ts = constants{7}; % Sample time (s)
    innerDyn_length = constants{16}; % Number of inner control loop iterations
    trajectory = constants{20}; % Trajectory type

    % Calculate angular frequency and height difference
    alpha = 2 * pi * f .* t;
    d_height = height_f - height_i;

    % Initialize position vectors
    x = zeros(1, length(t));
    y = zeros(1, length(t));
    z = zeros(1, length(t));

    % Generate trajectory based on selected type
    switch trajectory
        case 1
            % Circular trajectory
            x = r .* cos(alpha);
            y = r .* sin(alpha);
            z = height_i + d_height * t / t(end);
        case 2
            % Spiral trajectory
            x = (r / 10 .* t + 2) .* cos(alpha);
            y = (r / 10 .* t + 2) .* sin(alpha);
            z = height_i + d_height * t / t(end);
        case 3
            % Linear trajectory
            x = 2 * t / 20 + 1;
            y = 2 * t / 20 - 2;
            z = height_i + d_height * t / t(end);
        case 4
            % Vertical trajectory
            x = r .* cos(alpha);
            y = 2 * t;
            z = height_i + d_height * t / t(end);
        otherwise
            % Default trajectory (oscillatory)
            x = r .* cos(alpha);
            y = r .* sin(alpha);
            z = height_i + 50 * d_height / t(end) * sin(t);
    end

    % Calculate velocity (first derivative)
    dx = [diff(x), 0]; % Append 0 for the last element
    dy = [diff(y), 0];
    dz = [diff(z), 0];

    x_dot = dx * (1 / (Ts * innerDyn_length));
    y_dot = dy * (1 / (Ts * innerDyn_length));
    z_dot = dz * (1 / (Ts * innerDyn_length));

    % Calculate acceleration (second derivative)
    ddx = [diff(x_dot), 0];
    ddy = [diff(y_dot), 0];
    ddz = [diff(z_dot), 0];

    x_dot_dot = ddx * (1 / (Ts * innerDyn_length));
    y_dot_dot = ddy * (1 / (Ts * innerDyn_length));
    z_dot_dot = ddz * (1 / (Ts * innerDyn_length));

    %% Generate the reference yaw angles
    psi = zeros(1, length(x));
    psiInt = zeros(1, length(x));
    psi(1) = atan2(y(1), x(1)) + pi / 2;
    psi(2:end) = atan2(dy(2:end), dx(2:end));

    % Integrate yaw angles, handling angle wrapping
    dpsi = diff(psi);
    psiInt(1) = psi(1);
    for i = 2:length(psiInt)
        if dpsi(i-1) < -pi
            psiInt(i) = psiInt(i-1) + (dpsi(i-1) + 2 * pi);
        elseif dpsi(i-1) > pi
            psiInt(i) = psiInt(i-1) + (dpsi(i-1) - 2 * pi);
        else
            psiInt(i) = psiInt(i-1) + dpsi(i-1);
        end
    end
    psiInt = round(psiInt, 8);

    % Prepare reference outputs
    X_ref = [t', x'];
    X_dot_ref = [t', x_dot'];
    X_dot_dot_ref = [t', x_dot_dot'];
    Y_ref = [t', y'];
    Y_dot_ref = [t', y_dot'];
    Y_dot_dot_ref = [t', y_dot_dot'];
    Z_ref = [t', z'];
    Z_dot_ref = [t', z_dot'];
    Z_dot_dot_ref = [t', z_dot_dot'];
    psi_ref = [t', psiInt'];

end

%% Function to initialize constants
function constants = initial_constants()
    % Define constants
    sample_time = 0.1; % Sample time in seconds
    inner_control_loop_iterations = 10; % Number of inner control loop iterations
    trajectory_type = 1; % Default trajectory type

    % More constants can be added here as needed
    constants = {0, 0, 0, 0, 0, 0, sample_time, inner_control_loop_iterations, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, trajectory_type};
end
