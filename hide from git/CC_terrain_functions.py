import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import math as m

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.pose)
    # conf.enable_stream(rs.stream.depth)
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

def project_point(f,depth,position,roll_pitch_yaw):
    
    P1_matrix = np.zeros((1,4))
    P0_matrix = np.zeros((1,4))
    
    for x in range(640):
        for y in range(480):
            dist = depth.get_distance(x, y)
            P1_vector = np.array([x,y,dist,1])

            P1_matrix = np.vstack((P1_matrix,P1_vector))

            H = transformation_matrix_creator(position,roll_pitch_yaw)
            P0_vector = np.dot(H,P1_vector)
            P0_matrix = np.vstack((P0_matrix,P0_vector))

    return P0_matrix

#1. transform centerline of frame at origin to current frame
#   return the x and y values needed for depth values (projected centerline)
def project_centerline(position, roll_pitch_yaw):
    projected_CL = np.zeros((1,2))
    for y in range(480):
        x = 0
        P0 = np.array([[x],[y],[0],[1]])
        H = transformation_matrix_creator(position, roll_pitch_yaw)
        H_inv = np.linalg.inv(H)
        P1 = np.dot(H_inv, P0)
        P1_xy = np.array([np.asscalar(P1[0]),np.asscalar(P1[1])])
        projected_CL = np.vstack((projected_CL, P1_xy))
    return projected_CL

#2. take in x and y values from above and create the distance vectors from the current reference frame
#   return the transformed distance vectors from the origin

def distances_from_origin(f,p,position,roll_pitch_yaw):
    #depth_sensor = p.get_device().first_depth_sensor()
    #depth_scale = depth_sensor.get_depth_scale()

    points = project_centerline(position, roll_pitch_yaw)
    P1_matrix = np.zeros((1,4))
    P0_matrix = np.zeros((1,4))

    for row in points:
        x = row[0]
        y = row[1]
    
        depth = f.get_depth_frame()
        if not depth:
            dist = 0
            continue

        dist = depth.get_distance(x, y)
        P1_vector = np.array([x,y,dist,1])

        P1_matrix = np.vstack((P1_matrix,P1_vector))

        H = transformation_matrix_creator(position,roll_pitch_yaw)
        P0_vector = np.dot(H,P1_vector)
        P0_matrix = np.vstack((P0_matrix,P0_vector))

    return P0_matrix

#3. plot the transformed distance vector
def terrain_plot(P0_matrix):
    y = P0_matrix[:,1]
    z = P0_matrix[:,2]
    plt.plot(y,z)
    plt.show()

