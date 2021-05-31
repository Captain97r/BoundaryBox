classdef stlConverter
    %STLCONVERTER Contains methods for converting STL-files to both ASCII
    %representation and list of triangles
    
    methods(Static)
        function [norm, vert] = binaryToAscii()
            [filename, path] = uigetfile({'*.stl','STL Files (*.stl)';
                                           '*.*', 'All Files'});
            
            if (filename(end-3:end) ~= ".stl")
                disp("Invalid file format");
                return;
            end
            
            tic
            fileID = fopen(strcat(path, filename));
            binary = fread(fileID);

            faces_num = typecast(uint8([binary(81) binary(82) binary(83) binary(84)]), 'uint32');
            norm = zeros(faces_num, 3);
            vert = zeros(3, 3, faces_num);

            for face = 1:faces_num
                norm(face, 1) = typecast(uint8([binary(85 + (face - 1) * 50), ...
                                                binary(86 + (face - 1) * 50), ...
                                                binary(87 + (face - 1) * 50), ...
                                                binary(88 + (face - 1) * 50)]), ...
                                                'single');

                norm(face, 2) = typecast(uint8([binary(89 + (face - 1) * 50), ...
                                                binary(90 + (face - 1) * 50), ...
                                                binary(91 + (face - 1) * 50), ...
                                                binary(92 + (face - 1) * 50)]), ...
                                                'single');

               norm(face, 3) = typecast(uint8([binary(93 + (face - 1) * 50), ...
                                               binary(94 + (face - 1) * 50), ...
                                               binary(95 + (face - 1) * 50), ...
                                               binary(96 + (face - 1) * 50)]), ...
                                               'single');

                for v = 1:3
                    vert(v, 1, face) = typecast(uint8([ binary(85 + v * 12 + (face - 1) * 50), ...
                                                        binary(86 + v * 12 + (face - 1) * 50), ...
                                                        binary(87 + v * 12 + (face - 1) * 50), ...
                                                        binary(88 + v * 12 + (face - 1) * 50)]), ...
                                                        'single');

                    vert(v, 2, face) = typecast(uint8([ binary(89 + v * 12 + (face - 1) * 50), ...
                                                        binary(90 + v * 12 + (face - 1) * 50), ...
                                                        binary(91 + v * 12 + (face - 1) * 50), ...
                                                        binary(92 + v * 12 + (face - 1) * 50)]), ...
                                                        'single');

                    vert(v, 3, face) = typecast(uint8([ binary(93 + v * 12 + (face - 1) * 50), ...
                                                        binary(94 + v * 12 + (face - 1) * 50), ...
                                                        binary(95 + v * 12 + (face - 1) * 50), ...
                                                        binary(96 + v * 12 + (face - 1) * 50)]), ...
                                                        'single');

                end
            end
            fclose(fileID);
            toc
        end
        
        function [x, y, z, T] = asciiToTriangles(vert)
            tic
            size_vert = size(vert);
            faces_num = size_vert(3);
            points = size_vert(3) * size_vert(2);

            x = zeros(1, points);
            y = zeros(1, points);
            z = zeros(1, points);
            T = zeros(faces_num, 3);
            ind = 1;

            for face = 1:faces_num
                for p = 1:3
                    c_x = x == vert(p, 1, face);
                    c_y = y == vert(p, 2, face);
                    c_z = z == vert(p, 3, face);

                    c = c_x + c_y + c_z;

                    if (sum(c == 3) == 0)
                        x(ind) = vert(p, 1, face);
                        y(ind) = vert(p, 2, face);
                        z(ind) = vert(p, 3, face);
                        T(face, p) = ind;
                        ind = ind + 1;
                    else
                        found_p = find(c == 3);
                        T(face, p) = found_p;
                    end
                end
            end
            x(ind:end) = [];
            y(ind:end) = [];
            z(ind:end) = [];
            toc
        end
        
        function [x, y, z, T, norm] = binaryToTriangles()
            [norm, vert] = stlConverter.binaryToAscii();
            [x, y, z, T] = stlConverter.asciiToTriangles(vert);
        end  
    end
end

