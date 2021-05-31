[x, y, z, T, norm] = stlConverter.binaryToTriangles;
trimesh(T, x, y, z);
norm = norm * (-1);
hold on

% Построение траектории
[trajectory, point_list, pass_over] = BoundaryBox.boundary_box(T, x, y, z, norm);

% Подход инструмента
[tool_trajectory, tool_point_list] = BoundaryBox.tool_feed(trajectory, point_list, pass_over, 5);
plot3(tool_trajectory(1:end, 1), tool_trajectory(1:end, 2), tool_trajectory(1:end, 3));

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

%plot(trajectory(1:end, 1), trajectory(1:end, 3));

