#import pyrealsense2 as rs
import math as m
import numpy as np
import matplotlib.pyplot as plt
import random as r

#unsorted global variables
init_angle = 30 #replaced in each frame by imu data read
init_height= 18

#reassigns each value in matrix with a noisy value
def noise(matrix):
    mu, sigma = 0, 0.1 # mean and standard deviation
    s = np.random.normal(mu, sigma, 1000)
    n = np.size(s,0)
    for value in np.nditer(matrix, op_flags=['readwrite']):
        value[...] = value*s[r.randint(0,n)]
    return matrix

#makes a truth
def truth(): #some sort of blocky terrain
    z = np.linspace(0,20,21)
    y = np.random.randint(10, size = z.shape[0])
    z1 = np.repeat(z,2)[1:]
    y1 = np.repeat(y,2)[:-1]
    
    #plt.plot(z1,y1)
    #plt.show()
    
    return np.concatenate(([z1],[y1]), axis = 0)
    
def pointIsOnLine(slope, c, z, y): 
    if (y == ((slope * z) + c)):  
        return True;  
    return False;  

#makes a pointcloud of the truth terrain
truth = truth()
def pointcloud(init_angle, init_height, truth):
    pointcloud = np.array([])
    FOV = 57
    angle_min = init_angle-(FOV/2)
    angle_max = init_angle+(FOV/2)
    #c = init_displacement in z
    for angle in np.arange(angle_min, angle_max):
        slope = m.tan(m.radians(angle))
        i = 0
        for i in range(np.size(truth,1)):
            coordinate = truth[:,i]
            z = round(coordinate[0],1)
            y = round(coordinate[1],1)
            if (pointIsOnLine(slope,0,z,y)):
                pointcloud = np.append(pointcloud,[z, y], axis=0)
                i+=1
            else:
                i+=1
    return pointcloud

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.pose)
    prof = p.start(conf)

    return p

def position_data(position):
    return np.asarray([position.x, position.y, position.z])

def roll_pitch_yaw_calc(data):
    w = data.rotation.w
    x = -data.rotation.z
    y = data.rotation.x
    z = -data.rotation.y

    pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
    roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
    yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;

    return np.asarray([roll, pitch, yaw])

def transformation_matrix_creator(position, roll_pitch_yaw):
    r = roll_pitch_yaw[0]
    p = roll_pitch_yaw[1]
    y = roll_pitch_yaw[2]

    rotation = np.array([[m.cos(y)*m.cos(r)-m.cos(p)*m.sin(r)*m.sin(y), m.cos(y)*m.sin(r)+m.cos(p)*m.cos(r)*m.sin(y), m.sin(y)*m.sin(p)], [-m.sin(y)*m.cos(r)-m.cos(p)*m.sin(r)*m.cos(y), -m.sin(y)*m.sin(r)+m.cos(p)*m.cos(r)*m.cos(y), m.cos(y)*m.sin(p)], [m.sin(p)*m.sin(r), -m.sin(p)*m.cos(r), m.cos(p)]])

    Rd = np.hstack((rotation, position.reshape(3,1)))
    H = np.vstack((Rd, np.array((0,0,0,1))))

    return H
    
    
pointcloud = pointcloud(init_angle, init_height, truth)
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