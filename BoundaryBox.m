classdef BoundaryBox
    %STLCONVERTER Contains functions for trajectory generation
    
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
        %   trajectory  - array containing the chain of points which represents
        %                 trajectory of the actuator (required output of the algo)
        %   point_list  - object which represents trajectory of the actuator with
        %                 some additional information about points (auxiliary output 
        %                 for some further operations) 
        %   pass_over   - array which contains the lengths of each pass of the
        %                 trajectory
        function [trajectory, point_list, pass_over] = boundary_box(T, x, y, z, norm)
        tic
        
        %% Declaration and definition of variables
        pass_over = [];
        point_list = [];
        trajectory = [];
        current_pass = [];
        src = [];
        s = size(T);
        
        %% Main loop  
        % Since we don't divide surface it's redundant but 
        % DON'T TOUCH WHAT WORKS JUST FINE ¯\_(ツ)_/¯
        while (s(1) > 0)
            Tplane = T;
            e = mathHelper.get_edges(Tplane);

            % Tries to define what edge we should start from
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

            plane_equation = [0 0 0 0]; % [A B C D] - coefficients in equation of the form of Ax+By+Cz+D=0

            % TODO: we still don't automatically select the plane to cut with
            sec_plane = 2;% mathHelper.get_nearest_othogonal_plane([0 0 0]);
            plane_equation(sec_plane) = 2;

            min_val(sec_plane) = min_val(sec_plane) +10;
            
            % Now we create path based on the cutting plane and list of
            % triangles making up the original plane
            
            % TODO: we dont't automatically select the step of passes as
            % well as boundaries determination
            for slice = min_val(sec_plane):3:max_val(sec_plane)
                plane_equation(4) = -slice; 
                p = mathHelper.get_points(e, x, y, z, plane_equation);
                % Gets the sequence of points which form a continuous chain
                % in terms of positions of triangles in our original plane
                [p, list] = mathHelper.get_bounded_points(e, x, y, z, plane_equation, Tplane);

                % Unique function implementation here
                % Required to avoid double-precision inaccuracy, deletes
                % repeating points
                list_size = size(list);
                list_length = list_size(2);
                ptr = 1;
                while (ptr < list_length)
                    for i = list_length:-1:ptr + 1
                        if (ceil(list(ptr).points * 100000) == ceil(list(i).points * 100000))
                            list(i)= [];
                        end
                    end
                    list_size = size(list);
                    list_length = list_size(2);
                    ptr = ptr + 1;
                end
                
                % Begins to build trajectory array
                % First, defines the point we gonna start our pass from
                if (~isempty(trajectory))
                    pivot = trajectory(end, :);
                    min_dist = Inf;
                    min_ptr = -1;
                    for i = 1:list_length
                        current_point = list(i).points;
                        dist = mathHelper.get_distance(pivot, current_point);
                        if (dist < min_dist)
                            min_dist = dist;
                            min_ptr = i;
                        end
                    end
                    start_point = min_ptr;
                end
                
                % Runs algo which building path on the surface using a list
                % containing our chain of points
                list = BoundaryBox.path_finding(list, -1);
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                for i = 1:list_length
                    current_pass = [current_pass; list(i).points];
                end
               
                % Creates additional points in resulting trajectory to 
                % maintain a constant step between them
                [step_pass, point_list_step] = BoundaryBox.get_trajectory_with_constant_step(current_pass, list, T, x, y, z);
                
                % Moves points along normals to define the path of the
                % actuator (seems like redundant)
                [exp_pass, point_list_exp] = BoundaryBox.expand_trajectory(step_pass, point_list_step, norm, 5);
                
                % Smoothes normals to fix sudden moves of the actuator
                [filtered_norm] = BoundaryBox.norm_filter(exp_pass, step_pass);
                
                % Moves points along smoothed normals to define the path of 
                % the actuator
                [filtered_pass] = BoundaryBox.expand_trajectory_by_norm(step_pass, filtered_norm, 5); 
                
                
                % Drawing routine
                plot3(step_pass(1:end, 1), step_pass(1:end, 2), step_pass(1:end, 3));
                hold on
                plot3(filtered_pass(1:end, 1), filtered_pass(1:end, 2), filtered_pass(1:end, 3));
                hold on
                prev_angle = -1000;
                prev_norm = filtered_norm(1, :);
                for i=1:length(filtered_pass)
                     angle = mathHelper.get_angle(prev_norm, filtered_norm(i, :));
                     axis equal
                    if abs(angle - prev_angle) < 10
                        continue;
                    end
                    
                    segment = [step_pass(i, :); mathHelper.point_shift_vec(step_pass(i, :), filtered_norm(i, :), 5)]; 
                    plot3(segment(1:end, 1), segment(1:end, 2), segment(1:end, 3));
                    hold on
                    prev_angle = angle;
                    prev_norm = filtered_norm(i, :);
                end
                
                
                % Reverting every second pass
                if (~isempty(trajectory))
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
                
                pass_over = [pass_over, length(trajectory)];
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
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                %plot3(trajectory(1:end, 1), trajectory(1:end, 2), trajectory(1:end, 3));
                %plot3(trajectory(1:end, 1), trajectory(1:end, 2), trajectory(1:end, 3), '.');
                %p = unique(p, 'rows');
                %trajectory = [trajectory; p];

            end
            s = 0;  
            plot3(trajectory(1:end, 1), trajectory(1:end, 2), trajectory(1:end, 3));
            %plot3(src(1:end, 1), src(1:end, 2), src(1:end, 3));
            %axis equal
        end   
        toc
        end
        
        function path_list = path_finding(list, start_point)
        
            list_size = size(list);
            list_length = list_size(2);

            endpoints = [];

            % Create list of points, belonging to only one triangle
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
            
                % And place it at the beginning of the array
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
            
                % And place it at the beginning of the array
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
        
        function [trajectory, point_list] = expand_trajectory(trajectory, point_list, norm, dist)
            
            trajectory = [];
            % Get expanded list of points
            list_size = size(point_list);
            
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
                tri = list_expanded(end).inside_triangle(1);
            end
%             if (norm(tri, 3) < 0)
%                 norm(tri, :) = norm(tri, :) * (-1);
%             end
            list_expanded(end).points = mathHelper.point_shift_vec(list_expanded(end).points, norm(tri, :), dist);
            
            for i = 1:list_size(2)
                trajectory = [trajectory; list_expanded(i).points];
            end
            
        end
        
        function [result] = expand_trajectory_by_norm(trajectory, norm, dist)
            result = zeros(length(trajectory), 3);
            for i = 1:length(trajectory)
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
    %             distance_map = zeros(tr_len, 1);
    %             for i = 2:tr_len-1
    %                 total_len = mathHelper.get_distance(trajectory(i-1, :), trajectory(i, :)) + ...
    %                     + mathHelper.get_distance(trajectory(i, :), trajectory(i+1, :));
    % 
    %                 distance = mathHelper.get_distance(trajectory(i-1, :), trajectory(i+1, :));
    %                 distance_map(i) = total_len / distance;
    %             end

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
                
                %plot3(result(1:end, 1), result(1:end, 2), result(1:end, 3));
            end
                
            
            
%             window_size = 49;
%             for i = 1:length(trajectory) - window_size
%                 m = mean(trajectory(i:i+window_size-1, :));
%                 result(i + floor(window_size / 2), :) = m;
%             end
        end
        
        function [result] = norm_filter(trajectory, source_trajectory)
            
            result = trajectory(1:end, :) - source_trajectory(1:end, :);
            for i = 1:length(trajectory)
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
    %             distance_map = zeros(tr_len, 1);
    %             for i = 2:tr_len-1
    %                 total_len = mathHelper.get_distance(trajectory(i-1, :), trajectory(i, :)) + ...
    %                     + mathHelper.get_distance(trajectory(i, :), trajectory(i+1, :));
    % 
    %                 distance = mathHelper.get_distance(trajectory(i-1, :), trajectory(i+1, :));
    %                 distance_map(i) = total_len / distance;
    %             end

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
                    segment_list.points = p1;
                    tri = point_list(i).triangles;
                    segment_list.triangles = tri;
                    
                    while (abs(mathHelper.get_distance(p2, segment(end, :))) > step_treshold)
                        new_p = mathHelper.point_shift(segment(end, :), p2, step_treshold);
                        segment = [segment; new_p];
                        
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
        
        function [norm] = construct_new_norm(step, filtered, len)
            tr_size = size(step);
            tr_len = tr_size(1);
            norm = zeros(tr_len, 3, 2);
            
            for i = 1:tr_len
                norm(i, :, 1) = filtered(i, :);
                norm(i, :, 2) = mathHelper.point_shift(filtered(i, :), step(i, :), len);
            end
        end
        
        function acc = check_kinematics(trajectory)
            tr_size = size(trajectory);
            tr_length = tr_size(1);
            acc = zeros(1, tr_length - 2);
            dist = zeros(1, tr_length - 1);
            
            for i = 1:tr_length - 1
                x = [trajectory(i, 1), trajectory(i + 1, 1)];
                y = [trajectory(i, 2), trajectory(i + 1, 2)];
                z = [trajectory(i, 3), trajectory(i + 1, 3)];
                
                dist(i) = mathHelper.get_distance(trajectory(i, :), trajectory(i + 1, :));
                %p = polyfit(x,z,2);
                %acc(i) = p(1);
            end
            
            for i = 1:tr_length - 2
                x = [trajectory(i, 1), trajectory(i + 1, 1), trajectory(i + 2, 1)];
                y = [trajectory(i, 2), trajectory(i + 1, 2), trajectory(i + 2, 2)];
                z = [trajectory(i, 3), trajectory(i + 1, 3), trajectory(i + 2, 3)];
                
                t = linspace(0, dist(i) + dist(i+1), round((dist(i) + dist(i+1)) * 100000));
                if (length(t) < 3)
                    continue;
                end
                ptr = find(t <= dist(i));
                x_t = [linspace(x(1), x(2), ptr(end)) linspace(x(2), x(3), length(t) - (ptr(end)-1))];
                x_t(ptr(end)) = [];
                
                p = polyfit(t,x_t,2);
                acc(i) = p(1);
                
                f1 = polyval(p,t);

                plot([0 dist(i) dist(i)+dist(i+1)],x,'o')
                hold on
                plot(t,x_t)
                plot(t, f1,'r--')
                legend('x','x_t','f_t')
                hold off
                
            end
            
        end
    end
end