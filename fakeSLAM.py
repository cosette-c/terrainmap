#import pyrealsense2 as rs
import math as m
import numpy as np
import matplotlib.pyplot as plt
import random as r

#unsorted global variables
init_angle = 120 #replaced in each frame by imu data read
init_height= 18
init_x = 0

#reassigns each value in matrix with a noisy value
def noise(matrix):
    mu, sigma = 0, 0.1 # mean and standard deviation
    s = np.random.normal(mu, sigma, 1000)
    n = np.size(s,0)
    for value in np.nditer(matrix, op_flags=['readwrite']):
        value[...] = value*s[r.randint(0,n)]
    return matrix

#makes a truth
def truth(x_range, y_range): #some sort of blocky terrain
    x = np.linspace(0,x_range,x_range+1)
    y = np.random.randint(y_range, size = np.size(x))
    x1 = np.repeat(x,2)[1:]
    y1 = np.repeat(y,2)[:-1]
    i = 0
    for i in range(i, np.size(x1), 2):
    	x1[i] += 0.5
    	i += 2
    plt.plot(x1,y1)
    return np.concatenate(([x1],[y1]), axis = 0)

def get_intersect(a1, a2, b1, b2):
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return (float('inf'), float('inf'))
    return np.array([x/z, y/z])

# #makes a pointcloud of the truth terrain
def pointcloud_generator(init_angle, init_height, init_x, truth):
    pointcloud_x = np.array([])
    pointcloud_y = np.array([])
    FOV = 57
    angle_min = init_angle-(FOV/2)
    angle_max = init_angle+(FOV/2)
    origin = [init_x, init_height]
    
    i = angle_min
    j = i-angle_min

    for i in range(angle_min, angle_max):
        same_j = False
        slope = m.tan(m.radians(i))
        ray_point = [init_x+40, ((init_x+40)*slope)+18]
        plt.plot([0,ray_point[0]],[18,ray_point[1]])
        while same_j == False and j <= np.size(truth, 1)-1:
            j_start = [truth[0, j], truth[1, j]]
            j_end = [truth[0, j+1], truth[1, j+1]]
            intersect = get_intersect(j_start, j_end, origin, ray_point)
            if intersect[0]>j_end[0] or intersect[1]>j_end[1]:
                j += 1
            else:
                same_j = True
                pointcloud_x = np.append(pointcloud_x, intersect[0])
                pointcloud_y = np.append(pointcloud_y, intersect[1])

    return np.array([pointcloud_x, pointcloud_y])

def plot(x_max, y_max, init_angle, init_x, init_height, truth):
    pointcloud = pointcloud_generator(init_angle, init_height, init_x, truth)
    plt.plot(init_x, init_height, 'r+')
    plt.plot(pointcloud[0], pointcloud[1], 'ro')

    FOV = 57
    angle_min = init_angle-(FOV/2)
    angle_max = init_angle+(FOV/2)

    x_range = np.linspace(0,x_max,x_max*10+1)
    y_range = np.linspace(0,y_max,y_max*10+1)
    slope_min = m.tan(m.radians(angle_min))
    slope_max = m.tan(m.radians(angle_max))

    min_line = slope_min*x_range+init_height
    pos_min = min_line[min_line>=0]
    x_min = x_range[0:pos_min.size]
    plt.plot(x_min, pos_min)

    max_line = slope_max*x_range+init_height
    pos_max = max_line[max_line>=0]
    x_fov_max = x_range[0:pos_max.size]
    plt.plot(x_fov_max, pos_max)
    plt.xlim(-1, x_max)
    plt.ylim(-1, init_height+1)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

plot(50,4,init_angle, init_x, init_height, truth(50,4))

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.pose)
    prof = p.start(conf)

    return p

#transfer coordinate system
def position_data(position):
    return np.asarray([position.x, position.y, position.z])

# #transfer coordinate system
# def roll_pitch_yaw_calc(data):
#     w = data.rotation.w
#     x = -data.rotation.z
#     y = data.rotation.x
#     z = -data.rotation.y

#     pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
#     roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
#     yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;

#     return np.asarray([roll, pitch, yaw])

# #transfer coordinate system
# def transformation_matrix_creator(position, roll_pitch_yaw):
#     r = roll_pitch_yaw[0]
#     p = roll_pitch_yaw[1]
#     y = roll_pitch_yaw[2]

#     rotation = np.array([[m.cos(y)*m.cos(r)-m.cos(p)*m.sin(r)*m.sin(y), m.cos(y)*m.sin(r)+m.cos(p)*m.cos(r)*m.sin(y), m.sin(y)*m.sin(p)], [-m.sin(y)*m.cos(r)-m.cos(p)*m.sin(r)*m.cos(y), -m.sin(y)*m.sin(r)+m.cos(p)*m.cos(r)*m.cos(y), m.cos(y)*m.sin(p)], [m.sin(p)*m.sin(r), -m.sin(p)*m.cos(r), m.cos(p)]])

#     Rd = np.hstack((rotation, position.reshape(3,1)))
#     H = np.vstack((Rd, np.array((0,0,0,1))))

#     return H
    
    
#pointcloud = pointcloud(init_angle, init_height, truth)
def project_points(f,depth,position,roll_pitch_yaw, pointcloud):
    P1_matrix = np.zeros((1,4))
    P0_matrix = np.zeros((1,4))
    projected_points = np.array([])
    
    for row in pointcloud:
        z = truth[row,0]
        y = truth[row,1]
        P1_vector = np.array([0,y,z,1])

        P1_matrix = np.vstack((P1_matrix,P1_vector))

        H = transformation_matrix_creator(position,roll_pitch_yaw)
        P0_vector = np.dot(H,P1_vector)
        P0_matrix = np.vstack((P0_matrix,P0_vector))
        
        projected_points = np.append(projected_points, np.tranpsoe(P0_matrix), axis=0)

    return projected_points


