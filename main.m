[x, y, z, T, norm] = stlConverter.binaryToTriangles;
trimesh(T, x, y, z);
%norm = norm * (-1);
hold on

options = struct;
% Determines the method of path finding along one slice
%   1: based on chain of points on edges of triangles
%   2: simple finds the nearest point
options.path_finding_method = 2;

% Determines the method of slice construction between several actuator passes
%   1: stock (secant planes on equal distance)
%   2: equal distances between slices on surface
options.slice_construction_method = 2;

% Trajectory generation
[trajectory, point_list, pass_over] = BoundaryBox.find_shortest_path(T, x, y, z, norm, 0.1, 0.1, options);

% Tool feed
[tool_trajectory, tool_point_list] = BoundaryBox.tool_feed(trajectory, point_list, pass_over, 1);

% Results
plot3(tool_trajectory(1:end, 1), tool_trajectory(1:end, 2), tool_trajectory(1:end, 3));

% Accelerations and velocities
dstep = zeros(1, length(trajectory) - 1);
dout = zeros(1, length(trajectory) - 1);
v = zeros(1, length(trajectory) - 1);
a = zeros(1, length(trajectory) - 2);

for i = 1:length(trajectory) - 1
    dout(i) = mathHelper.get_distance(trajectory(i, :), trajectory(i+1, :));
    dstep(i) = mathHelper.get_distance(trajectory(i, :), trajectory(i+1, :));
    v(i) = dout(i);
end

for i = 1:length(trajectory) - 2
    a(i) = v(i+1) - v(i);
end
% 
hold off
figure(2)
plot(v)
xlabel('Время, с')
ylabel('Скорость, м/c');

figure(3)
plot(a)
xlabel('Время, c')
ylabel('Ускорение, м/с^2');

