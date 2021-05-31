classdef PointList < handle
    %MULTILIST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points = zeros(0, 3);
        triangles;
        inside_triangle;
    end
    
    methods
        
        function self = PointList()
            %MULTILIST Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function insertTriangles(self, tri)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            self.triangles = tri;
        end
        
        function insertPoints(self, p1, p2, p3)
            self.points = [p1 p2 p3];
        end
        
        function result = isEmpty(self)
            result = isequal(self.points, zeros(0, 3));
        end
        
        function unique(self)
            
        end
    end
end