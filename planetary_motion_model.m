% Gravitational constant
G = 6.67e-11;

% Time step (1 hour) and total number of steps
dt = 3600;
timesteps = 26280; % Total time = 3 years

% Number of bodies in the simulation
N = 6;

% --------------------- Figure setup ----------------------
figure(1);
axis equal;
hold on;
view(3); % Default 3D view, can be changed to 2d for top-down view
axis([-3e11 4e11 -3e11 4e11 -2e11 2e11]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Solar System Simulation');

% Trajectory paths and current position markers
h_path = gobjects(N, 1);
h_marker = gobjects(N, 1);

% One colour per planet
colors = {'y', 'b', 'g', 'c', 'm', 'r'}; 

% Create animated paths and markers for each body
for i = 1:N
    h_path(i) = animatedline('Color', colors{i}, 'LineWidth', 1); 
    h_marker(i) = plot3(0, 0, 0, 'o', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', colors{i}, ...
        'MarkerEdgeColor', 'k');
end

% ------------------- Initialisation -------------------------

% Net force on each planet
F_i = zeros(3,6);

% Store velocities and positions from the previous timestep
vel_old = zeros(3,6); 
disp_old = zeros(3,6); 

% Load initial conditions
for i = 1:6 
    vel_old(:,i) = [u(i), v(i), w(i)]; 
    disp_old(:,i) = [x(i), y(i), z(i)]; 
end

% Arrays for updated values
vel_new = zeros(3,6);
disp_new = zeros(3,6); 

% ---------------------- Time loop -----------------------
for ts = 1:timesteps

    % Reset forces at each timestep
    F_i = zeros(3,6);

    % Loop over each planet
    for planet = 1:6

        % Initialise empty 3D force array to store 
        % pairwise gravitational forces from all other bodies
        F_ij = zeros(3,6);

        for j = 1:6
            if planet ~= j % Skip over force on itself - not relevant

                % Relative position vector from planet to body j
                r_x = disp_old(1,j) - disp_old(1,planet);
                r_y = disp_old(2,j) - disp_old(2,planet);
                r_z = disp_old(3,j) - disp_old(3,planet);
                r = [r_x; r_y; r_z];

                % Distance between the two bodies
                abs_r = sqrt(r_x^2 + r_y^2 + r_z^2);

                % Newton's law of gravitation
                F_ij(:,j) = G * m(planet) * m(j) * r / abs_r^3;
            end
        end

        % Net force acting on the current planet
        F_i(:, planet) = sum(F_ij, 2);

        % Update velocity using gravitational acceleration
        vel_new(:, planet) = vel_old(:, planet) ...
            + F_i(:, planet) / m(planet) * dt;

        % Update position using kinematics (SUVAT)
        % S = u*t + 0.5*a*t^2
        disp_new(:, planet) = disp_old(:, planet) ...
            + vel_old(:, planet) * dt ...
            + F_i(:,planet) / (2 * m(planet)) * dt^2;
    end

    % Move to the next timestep (new)
    vel_old = vel_new;
    disp_old = disp_new;

    % ----- Plotting -----
    for planet = 1:6
        % Extend trajectory path line
        addpoints(h_path(planet), ...
            disp_old(1, planet), ...
            disp_old(2, planet), ...
            disp_old(3, planet));

        %  Update current position marker
        set(h_marker(planet), ...
            'XData', disp_old(1, planet), ...
            'YData', disp_old(2, planet));
    end

    drawnow limitrate; % Plot timestep and ensure constant output rate
end