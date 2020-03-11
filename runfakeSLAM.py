import fakeSLAM as f
import math as m
import numpy as np

#unsorted global variables
init_angle = 120 
init_height= 18
init_x = 0

camera_x = init_x
camera_y = init_height
camera_theta = 30
camera_position = np.array([camera_x, camera_y, camera_theta])

x_max = 50
y_max = 4
sigma_distance = 0.005
sigma_angle = 0.005


truth_world_frame = f.truth_world_frame(x_max, y_max)
pointcloud_world_frame = f.pointcloud_world_frame(init_angle, init_height, init_x, truth_world_frame, x_max, y_max)
pointcloud_camera_frame = f.pointcloud_to_camera_frame(pointcloud_world_frame, camera_position)
error_world_frame = f.error_to_world_frame(pointcloud_camera_frame, sigma_angle, sigma_distance, camera_position)

f.plot(x_max, y_max, init_angle, init_x, init_height, truth_world_frame, pointcloud_world_frame, error_world_frame, pointcloud_camera_frame)