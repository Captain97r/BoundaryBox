classdef PointList < handle
    
    properties
        points = zeros(0, 3);
        triangles;
        inside_triangle;
    end
    
    methods
        
        function self = PointList()
        end
        
        function insertTriangles(self, tri)
            self.triangles = tri;
        end
        
        function insertPoints(self, p1, p2, p3)
            self.points = [p1 p2 p3];
        end
        
        function result = isEmpty(self)
            result = isequal(self.points, zeros(0, 3));
        end
    end
end