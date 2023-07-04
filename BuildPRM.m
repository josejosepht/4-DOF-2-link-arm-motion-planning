% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = BuildPRM(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
adjacency=zeros(num_samples);
samples = SampleJointAngles(q_min,q_max,num_samples);

kdTree= KDTreeSearcher(samples);

% Initialize adjacency matrix to be a square zero matrix of length of
% num_samples
adjMatrix = zeros(num_samples);

%Replace points in samples with collision-free configurations(again uniformly randomly sampled)
for i=1:length(samples)
    while check_collision(robot,samples(i,:),link_radius,sphere_centers,sphere_radii)
        s=[];
        for j=1:4
            qtemp = unifrnd(q_min(j),q_max(j));
            s=[s qtemp];
        end
        samples(i,:) = s;
    end
end

% Compute the k-nearest neighboring edges between samples
for i = 1:num_samples
    % For each sample i, calculate k nearest neighbour configuration
    [idx, dist] = knnsearch(kdTree, samples(i,:), 'K', num_neighbors+1);
    
    %checking for collision free edges between neighbours
    for j = 2:num_neighbors+1
        if ~check_edge(robot, samples(i,:), samples(idx(j),:), link_radius, sphere_centers, sphere_radii)
            % Add the edge to the adjacency matrix
            adjMatrix(i,idx(j)) = dist(j);
            adjMatrix(idx(j),i) = dist(j);  
        end
    end
end
adjacency = adjMatrix;

end