% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = FindCollisionFreePath(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
% do a knn search to find the nearest elements in graph to q_start and
% q_goal
[idx,d]=knnsearch(samples,[q_start;q_goal],K=1);

%convert the adjacency matrix to a graph
G = graph(adjacency);

%use the shortestpath MATLAB function in the graph and set path_found=true
path_idx = shortestpath(G,idx(1),idx(2));
path_found = true;

%assign n to be size of shortest path and the start and end configurations
n = length(path_idx)+2;
path=zeros(n,4);

%iterate through the the shortest path and assign it to output path
%parameter
for i=1:length(path_idx)
    path(i+1,:) = samples(path_idx(i),:);
end

%assign start and goal configurations to output path parameter
path(1,:)=q_start;
path(end,:) = q_goal;
end