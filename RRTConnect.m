% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = RRTConnect(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
% Define the configuration space
q_limits = [q_min;q_max];

% Initialize the tree with the start configuration as the root node
start_node = q_start;
tree = start_node;

% Define the maximum number of iterations
max_iterations = 10000;

% Define the step size for extending the tree
step_size = 0.1;

% Define the goal region
goal_region = 0.1;

% Initialize the flag for reaching the goal configuration
path_found = false;

% RRT algorithm
for i = 1:max_iterations
    % Generate a random configuration
    q_rand = q_limits(1,:) + (q_limits(2,:) - q_limits(1,:)).*rand(1,4);
    
    % Find the nearest node in the tree
    dist = vecnorm(tree(:,1:4) - q_rand, 2, 2);
    [min_dist, nearest_node] = min(dist);
    q_near = tree(nearest_node,1:4);
    
    % Extend the tree towards the random configuration
    delta_q = step_size * (q_rand - q_near) / min_dist;
    q_new = q_near + delta_q;
    
    % Check if the new configuration is valid and collision-free
    if ~check_edge(robot,q_near,q_new, link_radius, sphere_centers, sphere_radii)
        % Add the new configuration to the tree as a new node
        new_node = q_new;
        tree = [tree; new_node];
        
        % Check if the goal configuration is reached
        if norm(q_new - q_goal) <= goal_region
            path_found = true;
            break;
        end
    end
end

kdTree= KDTreeSearcher(tree);
adjMatrix = zeros(length(tree));

num_neighbors = 10;
% Compute the edges between samples using the k-nearest neighbors algorithm
for i = 1:length(tree)
% Find the k-nearest neighbors of sample i
[idx, dist] = knnsearch(kdTree, tree(i,:), 'K', num_neighbors+1);
% For each neighbor, check if the edge is collision-free
for j = 2:num_neighbors+1
if ~check_edge(robot, tree(i,:), tree(idx(j),:), link_radius, sphere_centers, sphere_radii)
% Add the edge to the adjacency matrix
adjMatrix(i,idx(j)) = dist(j);
adjMatrix(idx(j),i) = dist(j);
end
end
end
adjacency = adjMatrix;

[idx,d]=knnsearch(tree,[q_start;q_goal],K=1);
G = graph(adjacency);
path_idx = shortestpath(G,idx(1),idx(2));
path_found = true;
n = length(path_idx)+2;
path=zeros(n,4);
for i=1:length(path_idx)
path(i+1,:) = tree(path_idx(i),:);
end
path(1,:)=q_start;
path(end,:) = q_goal;
end