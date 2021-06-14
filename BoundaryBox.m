classdef BoundaryBox
    %BoundaryBox Contains functions for trajectory generation
    
    methods(Static)
        %% Trajectory generator function
        % Input args:
        %   T - triangles' vertices array making up the plane, [T] = Nx3, 
        %       where N - the number of triangles and, accordingly, the length of 
        %       x, y and z arrays;
        %   x - an array of the stored x-coordinates of the triangles;
        %   y - an array of the stored y-coordinates of the triangles;
        %   z - an array of the stored z-coordinates of the triangles;
        %
        % Return values:
        %   trajectory       - array containing the chain of points which represents
        %                      trajectory of the actuator (required output of the algo)
        %   point_list       - object which represents trajectory of the actuator with
        %                      some additional information about points (auxiliary output 
        %                      for some further operations) 
        %   pass_over        - array which contains the lengths of each pass of the
        %                      trajectory
        %   expand_distance  - distance from surface to actuator
        %   tool_step        - distance between actuator passes
        %   secant_plane     - array of the form [A B C D] representing coefficients  
        %                      in plane equation of the form of Ax+By+Cz+D=0 
        function [trajectory, point_list, pass_over] = boundary_box(T, x, y, z, norm, expand_distance, tool_step, secant_plane)
        %% Declaration and definition of variables
        pass_over = [];
        point_list = [];
        trajectory = [];
        current_pass = [];
        src = [];
        s = size(T);
        
        % Determines the method of path finding
        %   1: based on chain of points on edges of triangles
        %   2: simple finds the nearest point
        path_finding_method = 2;
        
        % Determines the method of slice construction
        %   1: stock
        %   2: equal distances between slices
        slice_construction_method = 2;
        
        %% Main loop  
        % Since we don't divide surface it's redundant but 
        % DON'T TOUCH WHAT WORKS JUST FINE ¯\_(ツ)_/¯
        while (s(1) > 0)
            Tplane = T;
            e = mathHelper.get_edges(Tplane);

            % Tries to define what coordinates we should start from
            min_val = ones(1, 3) * inf;
            max_val = ones(1, 3) * -inf;
            for a = 1:length(e) * 2
                if (x(e(a)) > max_val(1))
                    max_val(1) = x(e(a));
                end
                if (y(e(a)) > max_val(2))
                    max_val(2) = y(e(a));
                end
                if (z(e(a)) > max_val(3))
                    max_val(3) = z(e(a));
                end

                if (x(e(a)) < min_val(1))
                    min_val(1) = x(e(a));
                end
                if (y(e(a)) < min_val(2))
                    min_val(2) = y(e(a));
                end
                if (z(e(a)) < min_val(3))
                    min_val(3) = z(e(a));
                end
            end

            plane_equation = secant_plane;
            
            min_val = min_val(secant_plane(1:3) ~= 0);
            max_val = max_val(secant_plane(1:3) ~= 0);
            center = (max(max_val) + min(min_val)) / 2;
            left_bound = center - (max(max_val) - min(min_val)) + ((max(max_val) - min(min_val)) * 0.01);
            right_bound = center + (max(max_val) - min(min_val));
            
            first_slice = 1;
            % Now we create path based on the cutting plane and list of
            % triangles making up the original plane
            if (slice_construction_method == 1)
                for slice = left_bound:tool_step:right_bound
                
                    plane_equation(4) = -slice; 

                    % Gets the sequence of points which form a continuous chain
                    % in terms of positions of triangles in our original plane
                    [p, list] = mathHelper.get_bounded_points(e, x, y, z, plane_equation, Tplane);

                    % Unique function implementation here
                    % Required to avoid double-precision inaccuracy, deletes
                    % repeating points
                    [list, list_length] = mathHelper.unique(list);

                    % Begins to build trajectory array
                    % First, defines the point we gonna start our pass from
                    % start_point = mathHelper.get_start_point(trajectory, list_length);

                    % Runs algo which builds the path on the surface using a list
                    % containing our chain of points

                    if (path_finding_method == 1)
                        list = BoundaryBox.path_finding(list, -1);
                    elseif (path_finding_method == 2)

                        test = zeros(list_length, 3);
                        for i = 1:list_length
                            test(i, :) = list(i).points - list(1).points;
                        end

                        if ~isempty(find(sum(test) ~= 0))
                            test = find(sum(test) ~= 0);
                            test = test(1);
                        end
                        min_p = inf;

                        index = -1;
                        % Finds min point
                        for i = 1:list_length
                            if (sum(list(i).points(test) < min_p) > 0)
                                min_p = list(i).points(test);
                                index = i;
                            end
                        end
                        if (index ~= -1)
                            temp = list(1);
                            list(1) = list(index);
                            list(index) = temp;
                        end

                        % Sorts array
                        for i = 1:list_length - 1
                            min_distance = inf;
                            index = -1;
                            for j = i+1:list_length
                                distance = mathHelper.get_distance(list(i).points, list(j).points);
                                if (distance < min_distance)
                                    min_distance = distance;
                                    index = j;
                                end
                            end
                            temp = list(index);
                            list(index) = list(i + 1);
                            list(i + 1) = temp;
                        end
                    end

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    for i = 1:list_length
                        current_pass = [current_pass; list(i).points];
                    end

                    % Creates additional points in resulting trajectory to 
                    % maintain a constant step between them
                    [step_pass, point_list_step] = BoundaryBox.get_trajectory_with_constant_step(current_pass, list, T, x, y, z);

                    % Moves points along normals to define the path of the
                    % actuator 
                    [exp_pass, point_list_exp] = BoundaryBox.expand_trajectory(step_pass, point_list_step, norm, expand_distance);

                    % Smoothes normals to fix sudden moves of the actuator
                    [filtered_norm] = BoundaryBox.norm_filter(exp_pass, step_pass);

                    % Moves points along smoothed normals to define the path of 
                    % the actuator
                    [filtered_pass] = BoundaryBox.expand_trajectory_by_norm(step_pass, filtered_norm, expand_distance); 


                    % Drawing normals routine
    %                 plot3(step_pass(1:end, 1), step_pass(1:end, 2), step_pass(1:end, 3));
    %                 hold on
    %                 plot3(filtered_pass(1:end, 1), filtered_pass(1:end, 2), filtered_pass(1:end, 3));
    %                 hold on
    %                 prev_angle = -1000;
    %                 prev_norm = filtered_norm(1, :);
    %                 for i=1:length(filtered_pass)
    %                      angle = mathHelper.get_angle(prev_norm, filtered_norm(i, :));
    %                      axis equal
    %                     if abs(angle - prev_angle) < 10
    %                         continue;
    %                     end
    %                     
    %                     segment = [step_pass(i, :); mathHelper.point_shift_vec(step_pass(i, :), filtered_norm(i, :), 5)]; 
    %                     plot3(segment(1:end, 1), segment(1:end, 2), segment(1:end, 3));
    %                     hold on
    %                     prev_angle = angle;
    %                     prev_norm = filtered_norm(i, :);
    %                 end


                    % Reverting every second pass
                    if (~isempty(trajectory) && ~isempty(filtered_pass))
                        d1 = mathHelper.get_distance(trajectory(end, :), filtered_pass(1, :));
                        d2 = mathHelper.get_distance(trajectory(end, :), filtered_pass(end, :));
                        if (d1 > d2)
                            filtered_pass = filtered_pass(end:-1:1, :);
                            step_pass = step_pass(end:-1:1, :);
                        end
                    end


                    % Oh what does code below means tell me pls
                    src = [src; step_pass];
                    trajectory = [trajectory; filtered_pass];

                    for i = 1:length(point_list_exp)
                        point_list = [point_list, point_list_exp(i)];
                    end

                    if (~isempty(filtered_pass))
                        pass_over = [pass_over, length(trajectory)];
                    end

                    v = zeros(1, length(filtered_pass) - 1);
                    a = zeros(1, length(filtered_pass) - 2);
                    for i = 1:length(filtered_pass) - 1
                        dout(i) = mathHelper.get_distance(filtered_pass(i, :), filtered_pass(i+1, :));
                        v(i) = dout(i);
                    end

                    for i = 1:length(filtered_pass) - 2
                        a(i) = v(i) - v(i+1);
                    end


                    list = [];
                    current_pass = [];

                    first_slice = 0;
                end
            elseif (slice_construction_method == 2)
                
                current_pass = [];
                plane_equation(4) = -left_bound;
                
                while (isempty(current_pass))

                        % Gets the sequence of points which form a continuous chain
                        % in terms of positions of triangles in our original plane
                        [p, list] = mathHelper.get_bounded_points(e, x, y, z, plane_equation, Tplane);

                        % Unique function implementation here
                        % Required to avoid double-precision inaccuracy, deletes
                        % repeating points
                        [list, list_length] = mathHelper.unique(list);

                        % Begins to build trajectory array
                        % First, defines the point we gonna start our pass from
                        % start_point = mathHelper.get_start_point(trajectory, list_length);
                        if (path_finding_method == 1)
                            list = BoundaryBox.path_finding(list, -1);
                        elseif (path_finding_method == 2)

                            test = zeros(list_length, 3);
                            for i = 1:list_length
                                test(i, :) = list(i).points - list(1).points;
                            end

                            if ~isempty(find(sum(test) ~= 0))
                                test = find(sum(test) ~= 0);
                                test = test(1);
                            end
                            min_p = inf;

                            index = -1;
                            % Finds min point
                            for i = 1:list_length
                                if (sum(list(i).points(test) < min_p) > 0)
                                    min_p = list(i).points(test);
                                    index = i;
                                end
                            end
                            if (index ~= -1)
                                temp = list(1);
                                list(1) = list(index);
                                list(index) = temp;
                            end

                            % Sorts array
                            for i = 1:list_length - 1
                                min_distance = inf;
                                index = -1;
                                for j = i+1:list_length
                                    distance = mathHelper.get_distance(list(i).points, list(j).points);
                                    if (distance < min_distance)
                                        min_distance = distance;
                                        index = j;
                                    end
                                end
                                temp = list(index);
                                list(index) = list(i + 1);
                                list(i + 1) = temp;
                            end
                        end
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                        for i = 1:list_length
                            current_pass = [current_pass; list(i).points];
                        end
                        plane_equation(4) = plane_equation(4) - tool_step;
                end
                        next_slice = BoundaryBox.find_next_slice_on_distance(T, e, x, y, z, norm, plane_equation, current_pass, list, tool_step);
            end
            s = 0;  
            %plot3(trajectory(1:end, 1), trajectory(1:end, 2), trajectory(1:end, 3));
            %plot3(src(1:end, 1), src(1:end, 2), src(1:end, 3));
            %axis equal
        end   
        end
        
        function [shortest_path, point_list, pass_over] = find_shortest_path(T, x, y, z, norm, expand_distance, tool_step)
            shortest_path = [];
            shortest_length = inf;
            
            %0z axis rotation
            for i = 0:pi/18:2*pi
                secant_plane = [cos(i) sin(i) 0 0];
                [trajectory, point_list, pass_over] = BoundaryBox.boundary_box(T, x, y, z, norm, expand_distance, tool_step, secant_plane);
                path_length = sum(mathHelper.get_distance(trajectory(2:end, :), trajectory(1:end-1, :)));
                path_over_distance = inf;
                if (length(passover) > 2)
                    path_over_distance = mathHelper.get_distance(trajectory(pass_over(1), :), trajectory(pass_over(3), :)) / 2;
                end
                
                if (path_length < shortest_length && path_over_distance < tool_step + 0.1 * tool_step)
                    shortest_path = [];
                    shortest_path = trajectory;
                    shortest_length = path_length;
                end
            end
            
            %0y axis rotation
            for i = 0:pi/18:2*pi
                secant_plane = [cos(i) 0 sin(i) 0];
                [trajectory, point_list, pass_over] = BoundaryBox.boundary_box(T, x, y, z, norm, expand_distance, tool_step, secant_plane);
                path_length = sum(mathHelper.get_distance(trajectory(2:end, :), trajectory(1:end-1, :)));
                if (path_length < shortest_length)
                    shortest_path = [];
                    shortest_path = trajectory;
                    shortest_length = path_length;
                end
            end
            
            %0x axis rotation
            for i = 0:pi/18:2*pi
                secant_plane = [0 cos(i) sin(i) 0];
                [trajectory, point_list, pass_over] = BoundaryBox.boundary_box(T, x, y, z, norm, expand_distance, tool_step, secant_plane);
                path_length = sum(mathHelper.get_distance(trajectory(2:end, :), trajectory(1:end-1, :)));
                if (path_length < shortest_length)
                    shortest_path = [];
                    shortest_path = trajectory;
                    shortest_length = path_length;
                end
            end
        end
        
        function path_list = path_finding(list, start_point)
        
            list_size = size(list);
            list_length = list_size(2);
            
            if (list_length == 0)
                path_list = PointList;
                return;
            end

            endpoints = [];

            % Creates list of points, belonging to only one triangle
            path_list(list_length) = PointList;
            for i = 1:list_length
                if (length(list(i).triangles) == 1)
                    endpoints = [endpoints i];
                end
            end

            % If this list doesn't empty, find point with lowest
            % z-coordinate (FIX IT!)
            if (~isempty(endpoints) && start_point ~= -1)

                min_dist = Inf;
                endp_index = -1;
                for i = 1:length(endpoints)
                    dist = mathHelper.get_distance(list(start_point).points, list(endpoints(i)).points);
                    if (dist < min_dist)
                        min_dist = dist;
                        endp_index = i;
                    end
                end
            
                % And places it at the beginning of the array
                if (endpoints(endp_index) ~= 1)
                    temp = list(endpoints(endp_index));
                    list(endpoints(endp_index)) = list(1);
                    list(1) = temp;
                    endpoints(1) = 1;
                end
            elseif (~isempty(endpoints) && start_point == -1)
                  
                min = Inf;
                index = -1;
                for i = 1:length(endpoints)
                    if (list(endpoints(i)).points(3) < min)
                        min = list(endpoints(i)).points(3);
                        index = i;
                    end
                end
            
                % The same
                if (endpoints(index) ~= 1)
                    temp = list(endpoints(index));
                    list(endpoints(index)) = list(1);
                    list(1) = temp;
                    endpoints(1) = 1;
                end  
            elseif (start_point ~= -1)
                temp = list(start_point);
                list(start_point) = list(1);
                list(1) = temp;
                % If it isn't, let's find one
            end
            

            
            % Necessary definition
            ptr = 2;
            path_list(1) = list(1);
            list(1) = [];
            dist = Inf;
            index = -1;
            is_started = 1;
            
            % Find the nearest point to the previous one and add to the
            % trajectory
            for i = 2:list_length
                if (length(path_list(i - 1).triangles) == 1 && ~is_started)
                    % Find the nearest endpoint
                    endpoints = [];
                    for k = 1:length(list)
                        if (length(list(k).triangles) == 1)
                            endpoints = [endpoints k];
                        end
                    end
                    for k = 1:length(endpoints)
                        cur_dist = mathHelper.get_distance(path_list(i - 1).points, list(endpoints(k)).points);
                        if (cur_dist < dist)
                            dist = cur_dist;
                            index = endpoints(k);
                        end
                    end
                    dist = Inf;
                    if (isempty(endpoints))
                        path_list(i) = list(end);
                        list(end) = [];
                    else
                        path_list(i) = list(index);
                        list(index) = [];
                    end
                    is_started = 1;
                    continue;
                end
                    
                for j = 1:list_length - i + 1
                    cur_dist = mathHelper.get_distance(path_list(i - 1).points, list(j).points);
                    if (cur_dist < dist)
                        dist = cur_dist;
                        index = j;
                    end
                end
            dist = Inf;
            path_list(i) = list(index);
            list(index) = [];
            is_started = 0;
            end
        end
        
        function next_slice = find_next_slice_on_distance(T, e, x, y, z, norm, current_sec_plane, current_slice, current_slice_list, distance)
            % TODO
            % 1. For each point find plane equation, perpendicular to current
            % secant plane and passing through its normal
            % 2. Calculate path length on surface which is equal to
            % distance given, and find end point
            
            current_slice_size = size(current_slice);
            current_slice_length = current_slice_size(1);
            
            next_slice = zeros(current_slice_length, 3);
            
            % Необходимо найти такую прямую, которая лежит в плоскости
            % сечения, и при этом перпендикулярна 
            % Не, надо просто найти уравнение плоскости, перпендикулярной
            % двум плоскостям - секущей и текущему треугольничку
            for i = 1:current_slice_length
                p1 = current_slice(i, :);
                
                % Нахождение нормали текущего треугольника
                if (length(current_slice_list(i).triangles) == 1)
                    current_norm = norm(current_slice_list(i).triangles(1), :);
                else
                    current_norm = (norm(current_slice_list(i).triangles(1), :) + norm(current_slice_list(i).triangles(2), :)) ./ 2;
                end
                %p2 = mathHelper.point_shift_vec(p1, current_norm, 1);
                
                % Находим ортогональную плоскость
                plane_norm = current_norm;
                
                ortogonal_plane = [current_norm(1), current_norm(2), current_norm(3), ...
                    -(current_norm(1) * p1(1)) - (current_norm(2) * p1(2)) - (current_norm(3) * p1(3))];
                
                % Направляющий вектор прямой, образованной пересечением
                % плоскостей
                dir_vec = [current_sec_plane(2) * ortogonal_plane(3) - current_sec_plane(3) * ortogonal_plane(2), ...
                           current_sec_plane(3) * ortogonal_plane(1) - current_sec_plane(1) * ortogonal_plane(3), ...
                           current_sec_plane(1) * ortogonal_plane(2) - current_sec_plane(2) * ortogonal_plane(1)];
                
                % Искомая плоскость, в которой будем искать расстояния
                sec_plane_ortogonal = [dir_vec(1), dir_vec(2), dir_vec(3), ...
                    -(dir_vec(1) * p1(1)) - (dir_vec(2) * p1(2)) - (dir_vec(3) * p1(3))];
                
                [points_on_ortogonal_plane, list] = mathHelper.get_bounded_points(e, x, y, z, sec_plane_ortogonal, T);
                
                [list, list_length] = mathHelper.unique(list);
                list_length = list_length + 1;
                list(list_length) = PointList();
                list(list_length).points = p1;
                list = mathHelper.sort_array_of_points(list, list_length);
                
                point_index = -1;
                
                orthogonal_path = zeros(list_length, 3);
                for k = 1:list_length
                    orthogonal_path(k, :) = list(k).points;
                    if orthogonal_path(k, :) == p1
                        point_index = k;
                    end
                end
                
                % Находим расстояние по поверхности до следующего прохода
                total_dist = 0;
                for k = point_index:list_length - 1
                    if (total_dist + mathHelper.get_distance(orthogonal_path(k, :), orthogonal_path(k+1, :))) > distance
                        point_index = k;
                        break;
                    end
                    total_dist = total_dist + mathHelper.get_distance(orthogonal_path(k, :), orthogonal_path(k+1, :));
                end
                
                % Получить направляющий вектор
                vec = mathHelper.vec_normalize(orthogonal_path(k+1, :) - orthogonal_path(k, :));
                % Отложить на направляющем векторе оставшееся расстояние
                endpoint = mathHelper.point_shift_vec(orthogonal_path(k, :), vec, distance - total_dist);
                next_slice(i, :) = endpoint;
            end
        end
        
        function [trajectory, point_list] = expand_trajectory(trajectory, point_list, norm, dist)
            
            trajectory = [];
            % Get expanded list of points
            list_size = size(point_list);
            
            if (list_size(2) == 0)
                return;
            end
            
            list_expanded(list_size(2)) = PointList();
            pts = point_list(1).points;
            tris = point_list(1).triangles;
            
            list_expanded(1).points = pts;
            list_expanded(1).triangles = tris;
            
            for i = 2:list_size(2) - 1
                pts = point_list(i).points;
                tris = point_list(i).triangles;
                in_tris = point_list(i).inside_triangle;
                
                list_expanded(i).points = pts;
                list_expanded(i).triangles = tris;
                list_expanded(i).inside_triangle = in_tris;
            end
            
            pts = point_list(end).points;
            tris = point_list(end).triangles;
            in_tris = point_list(end).inside_triangle;
            
            list_expanded(end).points = pts;
            list_expanded(end).triangles = tris;
            list_expanded(end).inside_triangle = in_tris;

            % For each member of expanded list do the shift in the
            % direction of norm vector
            list_size = size(list_expanded);
            
            % For the first one
            tri = list_expanded(1).triangles(1);
            list_expanded(1).points = mathHelper.point_shift_vec(list_expanded(1).points, norm(tri, :), dist);
            
            % For middle ones
            for i = 2:list_size(2) - 1
                tri = 0;
                if (isempty(list_expanded(i).triangles))
                    tri = list_expanded(i).inside_triangle;
                    try
                        list_expanded(i).points = mathHelper.point_shift_vec(list_expanded(i).points, norm(tri, :), dist);
                    catch
                        warning("Array is empty! Iteration: " + i);
                    end
                else
                    if (length(list_expanded(i).triangles) == 1)
                        tri = list_expanded(i).triangles(1);
                        list_expanded(i).points = mathHelper.point_shift_vec(list_expanded(i).points, norm(tri, :), dist);
                    else
                        tri1 = list_expanded(i).triangles(1);
                        tri2 = list_expanded(i).triangles(2);
                        norm_avg = (norm(tri1, :) + norm(tri2, :)) / 2;
                        list_expanded(i).points = mathHelper.point_shift_vec(list_expanded(i).points, norm_avg, dist);
                    end
                end
            end
            
            
            if (~isempty(list_expanded(end).triangles))
                tri = list_expanded(end).triangles(1);
            else
                try 
                    tri = list_expanded(end).inside_triangle(1);
                catch
                    warning();
                end
            end
%             if (norm(tri, 3) < 0)
%                 norm(tri, :) = norm(tri, :) * (-1);
%             end
            try 
                list_expanded(end).points = mathHelper.point_shift_vec(list_expanded(end).points, norm(tri, :), dist);
            catch
                warning();
            end
            
            for i = 1:list_size(2)
                trajectory = [trajectory; list_expanded(i).points];
            end
            
        end
        
        function [result] = expand_trajectory_by_norm(trajectory, norm, dist)
            result = zeros(length(trajectory), 3);
            trajectory_size = size(trajectory);
            for i = 1:trajectory_size(1)
                result(i, :) = mathHelper.point_shift_vec(trajectory(i, :), norm(i, :), dist); 
            end
        end
        
        function [result] = smooth_trajectory(trajectory, source_trajectory)
            result = trajectory;
            tr_size = size(trajectory);
            tr_len = tr_size(1);
            
            dist_threshold = 2;
            window_size = 11;
            is_ended = false;
            
            sizes = ones(tr_len, 1) * window_size;
            
            while ~is_ended

                for i = 2:tr_len-1
                    if (sizes(i) == 1)
                        continue;
                    end
                    
                    if (i - floor(sizes(i) / 2) <= 0)
                        m = mean(result(1:(i*2)-1, :));
                    elseif (i + floor(sizes(i) / 2) > tr_len)
                        m = mean(result(i-(tr_len-i):end, :));
                    else
                        m = mean(result(i-floor(sizes(i) / 2):i+floor(sizes(i) / 2), :));
                    end
                    result(i, :) = m;
                end
                
                
                is_ended = true;
                window_size = window_size + 2;
                sizes = ones(tr_len, 1);
                
                for i = 1:tr_len-1
                    dist_source = mathHelper.get_distance(source_trajectory(i, :), source_trajectory(i + 1, :));
                    dist_exp = mathHelper.get_distance(result(i, :), result(i + 1, :));
                    diff = dist_exp - dist_source;
                    if (abs(diff) > dist_threshold)
                        is_ended = false;
                        sizes(i) = window_size;
                    end
                end
                
                for i = 1:tr_len
                    if (sizes(i) > 1 && sizes(i-1) == 1)
                        k = 1;
                        while (sizes(i) - k > 1)
                            sizes(i - k) = sizes(i) - k;
                            k = k + 1;
                        end
                    end
                    if (sizes(i) > 1 && sizes(i+1) == 1)
                        k = 1;
                        while (sizes(i) - k > 1)
                            sizes(i + k) = sizes(i) - k;
                            k = k + 1;
                        end
                    end
                end
            end
                
        end
        
        function [result] = norm_filter(trajectory, source_trajectory)
            
            result = trajectory(1:end, :) - source_trajectory(1:end, :);
            trajectory_size = size(trajectory);
            for i = 1:trajectory_size(1)
                norm = sqrt(result(i, 1)*result(i, 1) + result(i, 2)*result(i, 2) + result(i, 3)*result(i, 3));
                result(i, 1) = result(i, 1) / norm;
                result(i, 2) = result(i, 2) / norm;
                result(i, 3) = result(i, 3) / norm;
            end
            
            tr_size = size(trajectory);
            tr_len = tr_size(1);
            
            angle_threshold = 2;
            window_size = 15;
            is_ended = false;
            
            sizes = ones(tr_len, 1) * window_size;  

            while ~is_ended

                for i = 2:tr_len-1
                    if (sizes(i) == 1)
                        continue;
                    end
                    
                    try
                        if (i - floor(sizes(i) / 2) <= 0)
                            m = mean(result(1:(i*2)-1, :));
                        elseif (i + floor(sizes(i) / 2) > tr_len)
                            m = mean(result(i-(tr_len-i):end, :));
                        else
                            m = mean(result(i-floor(sizes(i) / 2):i+floor(sizes(i) / 2), :));
                        end
                        result(i, :) = m;
                    catch
                        warning("Index exceeds array bounds, i: " + i);
                    end
                end
                
                is_ended = true;
                window_size = window_size + 2;
                sizes = ones(tr_len, 1);
                
                for i = 1:tr_len-1
                    
%                     angle = mathHelper.get_angle(result(i, :), result(i+1, :));
%                     
%                     if (abs(angle) > angle_threshold)
%                         is_ended = false;
%                         sizes(i) = window_size;
%                     end

                    p1 = mathHelper.point_shift_vec(source_trajectory(i, :), result(i, :), 5);
                    p2 = mathHelper.point_shift_vec(source_trajectory(i+1, :), result(i+1, :), 5);

                    if mathHelper.get_distance(p2, p1) > 1
                        is_ended = false;
                        sizes(i) = window_size;
                    end

                end
                
                for i = 2:tr_len-1
                    if (sizes(i) > 1 && sizes(i-1) == 1)
                        k = 1;
                        while (sizes(i) - k > 1 && i - k >= 1)
                            sizes(i - k) = sizes(i) - k;
                            k = k + 1;
                        end
                    end
                    if (sizes(i) > 1 && sizes(i+1) == 1)
                        k = 1;
                        while (sizes(i) - k > 1 && i + k <= length(sizes))
                            sizes(i + k) = sizes(i) - k;
                            k = k + 1;
                        end
                    end
                end
                
                %plot3(result(1:end, 1), result(1:end, 2), result(1:end, 3));
            end
                
            
            
%             window_size = 49;
%             for i = 1:length(trajectory) - window_size
%                 m = mean(trajectory(i:i+window_size-1, :));
%                 result(i + floor(window_size / 2), :) = m;
%             end
        end
        
        function [trajectory, point_list] = tool_feed(trajectory, point_list, pass_over, feed)
            
            k = 5;
            for i = 1:length(pass_over)-1
                
                
                vec1 = trajectory(pass_over(i), :) - trajectory(pass_over(i)-1, :);
                norm = sqrt(vec1(1)^2 + vec1(2)^2 + vec1(3)^2);
                vec1 = vec1 / norm;
                
                vec2 = trajectory(pass_over(i)+1, :) - trajectory(pass_over(i)+2, :);
                norm = sqrt(vec2(1)^2 + vec2(2)^2 + vec2(3)^2);
                vec2 = vec2 / norm;
                
                
                for j = 1:k
                    feed_step = feed / k;

                    point = trajectory(pass_over(i), :) + vec1 * feed_step * (k + 1 - j);
                    trajectory = [trajectory(1:pass_over(i), :); point; trajectory(pass_over(i)+1:end, :)];

                    tris = point_list(pass_over(i)+j-1).triangles;
                    point_list_object = PointList();
                    point_list_object.points = point;
                    point_list_object.triangles = tris;

                    point_list = [point_list(1:pass_over(i)) point_list_object point_list(pass_over(i)+1:end)];
                    
                    pass_over(i+1:end) = pass_over(i+1:end) + 1;
                    
                end
                for j = 1:k
                    point = trajectory(pass_over(i)+k+j, :) + vec2 * feed_step * (k + 1 - j);
                    trajectory = [trajectory(1:pass_over(i)+k+j-1, :); point; trajectory(pass_over(i)+k+j:end, :)];

                    tris = point_list(pass_over(i)+k+j).triangles;
                    point_list_object = PointList();
                    point_list_object.points = point;
                    point_list_object.triangles = tris;

                    point_list = [point_list(1:pass_over(i)+k+j-1) point_list_object point_list(pass_over(i)+k+j:end)];

                    pass_over(i+1:end) = pass_over(i+1:end) + 1;
                end
            end
            
            vec = trajectory(1, :) - trajectory(2, :);
            norm = sqrt(vec(1)^2 + vec(2)^2 + vec(3)^2);
            vec = vec / norm;

            for j = 1:k
                feed_step = feed / k;

                point = trajectory(j, :) + vec * feed_step * (j);
                trajectory = [point; trajectory];

                tris = point_list(j).triangles;
                point_list_object = PointList();
                point_list_object.points = point;
                point_list_object.triangles = tris;

                point_list = [point_list_object point_list];
            end
            
            
            vec = trajectory(end, :) - trajectory(end-1, :);
            norm = sqrt(vec(1)^2 + vec(2)^2 + vec(3)^2);
            vec = vec / norm;
            
            for j = 1:k
                feed_step = feed / k;

                point = trajectory(end-j+1, :) + vec * feed_step * (k + 1 - j);
                trajectory = [trajectory; point];

                tris = point_list(end-j+1).triangles;
                point_list_object = PointList();
                point_list_object.points = point;
                point_list_object.triangles = tris;

                point_list = [point_list point_list_object];
            end
            
        end
        
        function [trajectory_step, point_list_step] = get_trajectory_with_constant_step(trajectory, point_list, T, x, y, z)
            
            trajectory_step = [];
            step_norm = [];
            point_list_step = [];
            tr_size = size(trajectory);
            tr_length = tr_size(1);
            i = 1;
            step_treshold = 0.1;
            
            while i < tr_length
                dist = mathHelper.get_distance(trajectory(i, :), trajectory(i + 1, :));
                if (dist > step_treshold)
                    
                    p1 = trajectory(i, :);
                    p2 = trajectory(i+1, :);
                    
                    segment = p1;
                    segment_list = PointList();
                    try
                        segment_list.points = p1;
                        tri = point_list(i).triangles;
                        segment_list.triangles = tri;
                    catch
                        warning("PointList is empty!");
                    end
                    
                    while (abs(mathHelper.get_distance(p2, segment(end, :))) > step_treshold)
                        new_p = mathHelper.point_shift(segment(end, :), p2, step_treshold);
                        segment = [segment; new_p];
                        
                        try
                            new_p_object = PointList();
                            new_p_object.points = new_p;
                            tri = [];
                            new_p_object.triangles = tri;

                            x_t = x(T(point_list(i).triangles(1), :));
                            y_t = y(T(point_list(i).triangles(1), :));
                            z_t = z(T(point_list(i).triangles(1), :));
                            v1 = [x_t(1) y_t(1) z_t(1)];
                            v2 = [x_t(2) y_t(2) z_t(2)];
                            v3 = [x_t(3) y_t(3) z_t(3)];
                            inside = mathHelper.is_point_inside_triangle(new_p, v1, v2, v3);
                            if (inside)
                                new_p_object.inside_triangle = point_list(i).triangles(1);
                            elseif (length(point_list(i).triangles) > 1)
                                x_t = x(T(point_list(i).triangles(2), :));
                                y_t = y(T(point_list(i).triangles(2), :));
                                z_t = z(T(point_list(i).triangles(2), :));
                                v1 = [x_t(1) y_t(1) z_t(1)];
                                v2 = [x_t(2) y_t(2) z_t(2)];
                                v3 = [x_t(3) y_t(3) z_t(3)];
                                inside = mathHelper.is_point_inside_triangle(new_p, v1, v2, v3);
                                if (inside)
                                    new_p_object.inside_triangle = point_list(i).triangles(2);
                                end
                            else
                                new_p_object.inside_triangle = point_list(i).triangles(1);
                            end
                        
                            segment_list = [segment_list new_p_object];
                        
                        catch
                            warning("PointList object is empty!");
                        end
                        
                    end
                    trajectory_step = [trajectory_step; segment];
                    point_list_step = [point_list_step segment_list];
                    
                    i = i + 1;
                else
                    trajectory_step = [trajectory_step; trajectory(i, :)];
                    point_list_step = [point_list_step point_list(i)];
                    i = i + 1;
                end
            end
        end
        
        function progress_bar(total, count)
            persistent last_count;

            percentage = round((count / total) * 1000) / 10;
            last_percentage = round((last_count / total) * 1000) / 10;
            if (percentage ~= last_percentage)
                clc
                fprintf("%.1f%% completed\r", percentage);
                fprintf("[");

                total_sectors = 20;
                fill_sectors = floor(percentage / (100 / total_sectors));
                for i = 1:fill_sectors
                    fprintf("=");
                end
                if (fill_sectors < 20)
                    fprintf(">");
                end
                for i = fill_sectors+1:total_sectors-1
                    fprintf(" ");
                end
                fprintf("]\r");
            end
            last_count = count;
        end
    end
end