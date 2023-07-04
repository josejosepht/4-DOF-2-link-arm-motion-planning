% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_start, q_end -> 1x4 vectors denoting two ends of the
%                          configuration-space straight-line segment (edge)
%                          to check for collisions
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
%        resolution -> Scalar denoting how finely to check for collisions
% Output: in_collision -> Boolean, true if the robot is in collision with
%                         the given spherical obstacles on the
%                         straight-line segment (in configuration space)
%                         between q_start and q_end

function in_collision = check_edge(robot, q_start, q_end, link_radius, sphere_centers, sphere_radii, resolution)
    if nargin < 7
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution)';
    n = length(ticks);
    configs = repmat(q_start, n, 1) + repmat(q_end - q_start, n, 1) .* repmat(ticks, 1, length(q_start));
    
    in_collision = false;
    for i = 1:n
        if check_collision(robot, configs(i,:), link_radius, sphere_centers, sphere_radii)
            in_collision = true;
            break
        end
    end
end