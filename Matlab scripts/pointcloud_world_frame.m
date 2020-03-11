function [pointcloud_x, pointcloud_y] = pointcloud_world_frame(init_angle, init_height, init_x, x_max, y_max, truth_world_frame)

FOV = 57 ;
angle_min = init_angle - FOV/2 ; 
angle_max = init_angle + FOV/2 ;
origin = [init_x, init_height] ;

i = angle_min ;
j = i - angle_min ;

%% boolean ad

for i = angle_min:angle_max 
    same_j = false ; 
    slope = tan(degtorad(i)) ;
    ray_point = [init_x + x_max ((init_x + x_max)*slope + init_height)] ;
    while same_j = false && j<= size(truth_world_frame(1)
        j_start = [truth_world_frame(1,j) truth_world_frame(2,j)] ;
        j_end = [truth_world_frame(1,j+1) truth_world_frame(2,j+1)] ;
        intersect = polyxpoly(j_start j_end origin ray_point) ;
        
        
        
        
        
    end
    
end

end