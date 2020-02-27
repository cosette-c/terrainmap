#import pyrealsense2 as rs
import math as m
import numpy as np
import matplotlib.pyplot as plt
import random as r

def cartesian_to_polar(x,y):
    r = m.sqrt(x**2+y**2)
    if x == 0:
        theta = m.pi/2
    else:
        theta = m.atan(y/x)
    return(r,theta)

def polar_to_cartesian(r,theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return(x, y)

############################################################################
# still has a hardcoded variable, need a test function to see if transformations are happening correctly
############################################################################

def make_T(r, theta, phi):
    rotation = phi*m.pi/180 ;
    px = r*m.cos(theta)
    py = -r*m.sin(theta)

    T = np.array([[m.cos(rotation),  m.sin(rotation), 0, px], [-m.sin(rotation), m.cos(rotation), 0, py],[0, 0, 1, 0],[0, 0, 0, 1]])

    # px, py = polar_to_cartesian(r,theta)
    # T = np.array([[m.cos(theta),  m.sin(theta), 0, px], [-m.sin(theta), m.cos(theta), 0, py],[0, 0, 1, 0],[0, 0, 0, 1]])
    return T

#reassigns each value in matrix with a noisy value
def noise(matrix, sigma_angle, sigma_distance):
    distance_error = np.array([])
    angle_error = np.array([])

    mu_angle = 0 # mean and standard deviation
    mu_distance = 0
    if sigma_angle == 0:
        s_distance = np.random.normal(mu_distance, sigma_distance, 1000)
        s_angle = np.zeros(np.size(s_distance))
    elif sigma_distance == 0:
        s_distance = np.zeros(np.size(s_angle))
        s_angle = np.random.normal(mu_angle, sigma_angle, 1000)
    else:
        s_angle = np.random.normal(mu_angle, sigma_angle, 1000)
        s_distance = np.random.normal(mu_distance, sigma_distance, 1000)

    n = np.size(s_angle,0)

    for i in range(np.size(matrix,1)):
        distance = matrix[0,i]
        print(distance)
        angle = matrix[1,i]
        distance_error = np.append(distance_error, distance + s_distance[r.randint(0,n)])
        print(distance_error)
        angle_error = np.append(angle_error, angle + s_angle[r.randint(0,n)])

    return np.array([distance_error, angle_error])

def get_intersect(a1, a2, b1, b2):
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return (float('inf'), float('inf'))
    return np.array([x/z, y/z])

#makes a truth
def truth_world_frame(x_range, y_range): #some sort of blocky terrain
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

# #makes a pointcloud of the truth terrain
def pointcloud_world_frame(init_angle, init_height, init_x, truth_world_frame):
    pointcloud_x = np.array([])
    pointcloud_y = np.array([])
    FOV = 57
    angle_min = init_angle-(FOV/2)
    angle_max = init_angle+(FOV/2)
    origin = [init_x, init_height]
    
    i = angle_min
    j = i-angle_min

    for i in range(angle_min, angle_max, 1):
        same_j = False
        slope = m.tan(m.radians(i))
        ray_point = [init_x+40, ((init_x+40)*slope)+18]
        plt.plot([0,ray_point[0]],[18,ray_point[1]])
        while same_j == False and j <= np.size(truth_world_frame, 1)-1:
            j_start = [truth_world_frame[0, j], truth_world_frame[1, j]]
            j_end = [truth_world_frame[0, j+1], truth_world_frame[1, j+1]]
            intersect = get_intersect(j_start, j_end, origin, ray_point)
            if intersect[0]>j_end[0] or intersect[1]>j_end[1]:
                j += 1
            else:
                same_j = True
                pointcloud_x = np.append(pointcloud_x, intersect[0])
                pointcloud_y = np.append(pointcloud_y, intersect[1])

    return np.array([pointcloud_x, pointcloud_y])

def pointcloud_to_camera_frame(pointcloud_world_frame, camera_position):
    pointcloud_camera_frame_r = np.array([])
    pointcloud_camera_frame_theta = np.array([])
    T_to_camera = make_T(camera_position[0], camera_position[1], camera_position[2])

    for i in range(np.size(pointcloud_world_frame,1)):
        x = pointcloud_world_frame[0,i]
        y = pointcloud_world_frame[1,i]
        r,theta = cartesian_to_polar(x,y)
        p1_world = np.array([[x], [y], [0], [1]])
        po_camera = np.dot(T_to_camera, p1_world)
        r_camera, theta_camera = cartesian_to_polar(po_camera[0,0],po_camera[1,0])
        pointcloud_camera_frame_r = np.append(pointcloud_camera_frame_r, r_camera)
        pointcloud_camera_frame_theta = np.append(pointcloud_camera_frame_theta, theta_camera)

    return np.array([pointcloud_camera_frame_r, pointcloud_camera_frame_theta])

def error_to_world_frame(pointcloud_camera_frame, sigma_angle, sigma_distance, camera_position):
    error_world_frame_x = np.array([])
    error_world_frame_y = np.array([])
    error_camera_frame = noise(pointcloud_camera_frame, sigma_angle, sigma_distance)
    T_to_camera = make_T(camera_position[0], camera_position[1], camera_position[2])

    for i in range(np.size(error_camera_frame,1)):
        r = error_camera_frame[0,i]
        theta = error_camera_frame[1,i]
        x, y = polar_to_cartesian(r,theta)
        po_camera = np.array([[x], [y], [0], [1]])
        p1_world = np.dot(np.linalg.inv(T_to_camera), po_camera)
        error_world_frame_x = np.append(error_world_frame_x, p1_world[0,0])
        error_world_frame_y = np.append(error_world_frame_y, p1_world[1,0])

    return np.array([error_world_frame_x, error_world_frame_y])

def plot(x_max, y_max, init_angle, init_x, init_height, truth, pointcloud_world_frame, error_world_frame, pointcloud_camera_frame):
    plt.plot(init_x, init_height, 'r+') # camera position
    plt.plot(pointcloud_world_frame[0], pointcloud_world_frame[1], 'bo') # pointcloud

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
    plt.plot(x_min, pos_min) # FOV minimum

    max_line = slope_max*x_range+init_height
    pos_max = max_line[max_line>=0]
    x_fov_max = x_range[0:pos_max.size]
    plt.plot(x_fov_max, pos_max) # FOV maximum

    #plt.plot(pointcloud_camera_frame[0], pointcloud_camera_frame[1], 'g*')

    plt.plot(error_world_frame[0], error_world_frame[1], 'ro') # noisy points

    plt.xlim(-1, x_max)
    plt.ylim(-1, init_height+1)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

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
        #P0_vector = np.dot(H,P1_vector)
        P0_matrix = np.vstack((P0_matrix,P0_vector))
        
        projected_points = np.append(projected_points, np.tranpsoe(P0_matrix), axis=0)

    return projected_points


