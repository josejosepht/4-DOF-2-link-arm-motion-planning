% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = SmoothenPath(robot, path, link_radius, sphere_centers, sphere_radii)
smoothed_path=path;
for i=1:length(smoothed_path)
    q = smoothed_path(i,:);
    if q==path(end,:)
        break
    end
    for j=[length(smoothed_path):-1:1]
        if i~=j
            if ~check_edge(robot, smoothed_path(i,:), smoothed_path(j,:), link_radius, sphere_centers, sphere_radii,25)
                smoothed_path=[smoothed_path(1:i,:);smoothed_path(j:end,:)]
            end
        end
    end
end
end