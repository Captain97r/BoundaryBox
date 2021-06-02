classdef mathHelper
    %STLCONVERTER Contains methods for converting STL-files to both ASCII
    %representation and list of triangles
    
    methods(Static)
        function e = get_edges(T)
            size_T = size(T);
            e = zeros(size_T(1) * 3, 2);
            for i = 1:size_T(1)
                e((i - 1) * 3 + 1, :) = [T(i, 1) T(i, 2)];
                e((i - 1) * 3 + 2, :) = [T(i, 1) T(i, 3)];
                e((i - 1) * 3 + 3, :) = [T(i, 2) T(i, 3)];
            end
            e = unique(e, 'rows');
        end
        
        function a = get_nearest_othogonal_plane(norm)
            vec = [1 0 0];
            alpha1 = (mod(mathHelper.get_angle(norm, vec), 180)) - 90;
            a1 = mod(mathHelper.get_angle(norm, vec), 180);
            vec = [0 1 0];
            alpha2 = (mod(mathHelper.get_angle(norm, vec), 180)) - 90;
            a2 = mod(mathHelper.get_angle(norm, vec), 180);
            vec = [0 0 1];
            alpha3 = (mod(mathHelper.get_angle(norm, vec), 180)) - 90;
            a3 = mod(mathHelper.get_angle(norm, vec), 180);
            if (a2 == 0 && a1 ~= 0)
                a = 1;
            elseif (a2 == 0 && a3 ~= 0)
                a = 3;
            else 
                a = 2;
            end
                a = 2;
        end
        
        function alpha = get_angle(vec1, vec2)
            len_vec1 = sqrt(vec1(1) * vec1(1) + vec1(2) * vec1(2) + vec1(3) * vec1(3));
            len_vec2 = sqrt(vec2(1) * vec2(1) + vec2(2) * vec2(2) + vec2(3) * vec2(3));
            alpha = (acos(sum(vec1 .* vec2) / (len_vec1 * len_vec2))) * 180 / pi;
        end
        
        function points = get_points(e, x, y, z, plane_eq)

            A = plane_eq(1);
            B = plane_eq(2);
            C = plane_eq(3);
            D = plane_eq(4);

            points = zeros(length(e), 3);
            ptr = 1;

            for i = 1:length(e)
                % Направляющий вектор
                m = x(e(i, 2)) - x(e(i, 1));
                n = y(e(i, 2)) - y(e(i, 1));
                p = z(e(i, 2)) - z(e(i, 1));

                t = (-A*x(e(i, 1)) - B*y(e(i, 1)) - C*z(e(i, 1)) - D) / (A*m + B*n + C*p);
                pts(i, 1) = x(e(i, 1)) + m*t;
                pts(i, 2) = y(e(i, 1)) + n*t;
                pts(i, 3) = z(e(i, 1)) + p*t;

                x_b = sort([x(e(i, 1)) x(e(i, 2))]);
                y_b = sort([y(e(i, 1)) y(e(i, 2))]);
                z_b = sort([z(e(i, 1)) z(e(i, 2))]);

                if (pts(i, 1) >= x_b(1) && pts(i, 1) <= x_b(2) && pts(i, 2) >= y_b(1) && pts(i, 2) <= y_b(2) && pts(i, 3) >= z_b(1) && pts(i, 3) <= z_b(2))   
                    points(ptr, :) = [pts(i, 1) pts(i, 2) pts(i, 3)];
                    ptr = ptr + 1;
                end
            end
            points(ptr:end, :) = [];
        end
        
        function [points, list] = get_bounded_points(e, x, y, z, plane_eq, Tplane)

            list(length(e)) = PointList();
            A = plane_eq(1);
            B = plane_eq(2);
            C = plane_eq(3);
            D = plane_eq(4);

            points = zeros(length(e), 3);
            ptr = 1;

            for i = 1:length(e)
                % Направляющий вектор
                m = x(e(i, 2)) - x(e(i, 1));
                n = y(e(i, 2)) - y(e(i, 1));
                p = z(e(i, 2)) - z(e(i, 1));

                % Defining if point belongs to a edge
                t = (-A*x(e(i, 1)) - B*y(e(i, 1)) - C*z(e(i, 1)) - D) / (A*m + B*n + C*p);
                pts(i, 1) = x(e(i, 1)) + m*t;
                pts(i, 2) = y(e(i, 1)) + n*t;
                pts(i, 3) = z(e(i, 1)) + p*t;

                % Get boundaries
                x_b = sort([x(e(i, 1)) x(e(i, 2))]);
                y_b = sort([y(e(i, 1)) y(e(i, 2))]);
                z_b = sort([z(e(i, 1)) z(e(i, 2))]);

                if (pts(i, 1) >= x_b(1) && pts(i, 1) <= x_b(2) && pts(i, 2) >= y_b(1) && pts(i, 2) <= y_b(2) && pts(i, 3) >= z_b(1) && pts(i, 3) <= z_b(2))   
                    points(ptr, :) = [pts(i, 1) pts(i, 2) pts(i, 3)];
                    
                    % Find triangles which these points
                    t1 = ceil(find(Tplane' == e(i, 1)) / 3);
                    t2 = ceil(find(Tplane' == e(i, 2)) / 3);
                    triangles = intersect(t1, t2);
                    
                    list(ptr).insertPoints(pts(i, 1), pts(i, 2), pts(i, 3));
                    list(ptr).insertTriangles(triangles);
                    
                    ptr = ptr + 1;
                end
            end
            points(ptr:end, :) = [];
            list(ptr:end) = [];
        end
        
        function N = get_norm(x, y, z)
            vx1 = x(1) - x(2);
            vy1 = y(1) - y(2);
            vz1 = z(1) - z(2);
            vx2 = x(2) - x(3);
            vy2 = y(2) - y(3);
            vz2 = z(2) - z(3);

            n(1) = vy1 * vz2 - vz1 * vy2;
            n(2) = vz1 * vx2 - vx1 * vz2;
            n(3) = vx1 * vy2 - vy1 * vx2;


            nrmsqrt = sqrt((vy1*vz2-vz1*vy2)^2 + (vz1*vx2-vx1*vz2)^2 + (vx1*vy2-vy1*vx2)^2);
            N(1) = n(1) / nrmsqrt;
            N(2) = n(2) / nrmsqrt;
            N(3) = n(3) / nrmsqrt;
        end
        
        function l = get_distance(a, b)
            l = sqrt((a(:, 1) - b(:, 1)).^2 + (a(:, 2) - b(:, 2)).^2 + (a(:, 3) - b(:, 3)).^2);
        end
        
        function S = get_square(x, y, z)
            ab = [x(2) - x(1) y(2) - y(1) z(2) - z(1)];
            bc = [x(3) - x(2) y(3) - y(2) z(3) - z(2)];
            ac = [x(3) - x(1) y(3) - y(1) z(3) - z(1)];

            l_ab = sqrt(ab(1) * ab(1) + ab(2) * ab(2) + ab(3) * ab(3));
            l_bc = sqrt(bc(1) * bc(1) + bc(2) * bc(2) + bc(3) * bc(3));
            l_ac = sqrt(ac(1) * ac(1) + ac(2) * ac(2) + ac(3) * ac(3));

            p = 0.5 * (l_ab + l_bc + l_ac);
            S = sqrt(p * (p - l_ab) * (p - l_ac) * (p - l_bc));

        end
        
        function l = get_vec_len(v)
            l = sqrt(v(1) * v(1) + v(2) * v(2) + v(3) * v(3));
        end
        
        function p = point_shift(p1, p2, dist)
            m = p2(1) - p1(1);
            n = p2(2) - p1(2);
            k = p2(3) - p1(3);
            
            norm = sqrt(m*m + n*n + k*k);
            m = m / norm;
            n = n / norm;
            k = k / norm;
            
            p(1) = p1(1) + m * dist;
            p(2) = p1(2) + n * dist;
            p(3) = p1(3) + k * dist;
        end
        
        function p = point_shift_vec(p1, vec, dist)
            
            norm = sqrt(vec(1)*vec(1) + vec(2)*vec(2) + vec(3)*vec(3));
            vec(1) = vec(1) / norm;
            vec(2) = vec(2) / norm;
            vec(3) = vec(3) / norm;
            
            p(1) = p1(1) + vec(1) * dist;
            p(2) = p1(2) + vec(2) * dist;
            p(3) = p1(3) + vec(3) * dist;
        end
        
        function inside = is_point_inside_triangle(P, A, B, C)
            
            inside = 0;
            
            AB = sqrt( (A(1)-B(1))*(A(1)-B(1)) + (A(2)-B(2))*(A(2)-B(2)) + (A(3)-B(3))*(A(3)-B(3)) );
            BC = sqrt( (B(1)-C(1))*(B(1)-C(1)) + (B(2)-C(2))*(B(2)-C(2)) + (B(3)-C(3))*(B(3)-C(3)) );
            CA = sqrt( (A(1)-C(1))*(A(1)-C(1)) + (A(2)-C(2))*(A(2)-C(2)) + (A(3)-C(3))*(A(3)-C(3)) );

            AP = sqrt( (P(1)-A(1))*(P(1)-A(1)) + (P(2)-A(2))*(P(2)-A(2)) + (P(3)-A(3))*(P(3)-A(3)) );
            BP = sqrt( (P(1)-B(1))*(P(1)-B(1)) + (P(2)-B(2))*(P(2)-B(2)) + (P(3)-B(3))*(P(3)-B(3)) );
            CP = sqrt( (P(1)-C(1))*(P(1)-C(1)) + (P(2)-C(2))*(P(2)-C(2)) + (P(3)-C(3))*(P(3)-C(3)) );
            diff = (mathHelper.triangle_square(AP,BP,AB) + mathHelper.triangle_square(AP,CP,CA)+ ... 
                mathHelper.triangle_square(BP,CP,BC)) - mathHelper.triangle_square(AB,BC,CA);
            if (abs(diff) < 1e-10) 
                inside = 1;
            end
            
        end
        
        function sq = triangle_square(a, b, c)
            p=(a+b+c)/2;
            sq_squared = p *(p-a)*(p-b)*(p-c);
            if (sq_squared < 0)
                sq = 0;
            else
                sq = sqrt(sq_squared);
            end
        end
        
    end
end