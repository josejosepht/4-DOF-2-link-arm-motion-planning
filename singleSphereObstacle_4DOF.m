% You must run startup_rvc from Peter Corke Robotics Toolbox FIRST before running the below

% Create robot
robot = create_robot();

% Start and goal configuration
q_start = [0 -pi/4 0 -pi/4];
q_goal = [0 -3 0 -3];
% Minimum and maximum joint angles for each joint
q_min = [-pi/2 -pi 0 -pi];
q_max = [pi/2 0 0 0];
% Radius of each robot link's cylindrical body
link_radius = 0.03;

% Set up spherical obstacle
sphere_center = [0.5 0 0];
sphere_radius = 0.25;

% Plot robot and obstacle
robot.plot(q_start);
hold on;	
draw_sphere(sphere_center,sphere_radius);

% Parameters for PRM
num_samples = 100;
num_neighbors = 5;

[samples, adjacency] = BuildPRM(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_center, sphere_radius);

[path, path_found] = FindCollisionFreePath(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_center, sphere_radius);

if path_found
    fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
    disp(path);
    robot.plot(interpolate_path(path), 'fps', 10);
else
    disp('No path found.');
end

% Use the RRT algorithm to find a path from q_start to q_goal
[path, path_found] = RRTConnect(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius);
% Visualize the trajectory, if one is found
if path_found
    fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
    disp(path);
    % If trajectory is found, smooth the trajectory
    smoothed_path = SmoothenPath(robot, path, link_radius, sphere_center, sphere_radius);
    % Visualize the smoothed trajectory
    fprintf('Smoothed path found with %d intermediate waypoints:\n', size(smoothed_path, 1) - 2);
    disp(smoothed_path);
    robot.plot(interpolate_path(smoothed_path), 'fps', 10);
else
    disp('No path found.');
end



% Create a 4-DOF arm with 2 links
function robot = create_robot()
    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);    
    robot = SerialLink(L, 'name', 'robot');
end

% Draw a sphere with specified center position and radius
function draw_sphere(position, radius)
    [X,Y,Z] = sphere;
    X = X * radius;
    Y = Y * radius;
    Z = Z * radius;
    X = X + position(1);
    Y = Y + position(2);
    Z = Z + position(3);
    surf(X,Y,Z);
end

% Given a path consisting of configuration waypoints,
% interpolates the waypoints to give a less abrupt trajectory.
% Consecutive pairs of waypoints are interpolated along a straight line
% in configuration space, where the newly generated points are
% less than max_velocity apart in configuration space.
% This helper function is primarily used for visualization purposes.
function trajectory = interpolate_path(path, max_velocity)
    if nargin < 2
        max_velocity = 0.05;
    end
    trajectory = [path(1,:)];
    for i = 2:size(path, 1)
        vec = path(i,:) - path(i-1,:);
        num_ticks = ceil(norm(vec) / max_velocity);
        ticks = linspace(0, 1, num_ticks + 1)';
        segment = repmat(path(i-1,:), num_ticks, 1) + repmat(ticks(2:end,:), 1, length(path(i,:))) .* repmat(vec, num_ticks, 1);
        trajectory = [trajectory; segment];
    end
end