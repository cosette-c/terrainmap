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
def truth(z_range, y_range): #some sort of blocky terrain
    z = np.linspace(0,z_range,z_range+1)
    y = np.random.randint(y_range, size = z.shape[0])
    z1 = np.repeat(z,2)[1:]
    y1 = np.repeat(y,2)[:-1]
    
    plt.plot(z1,y1)
    
    return np.concatenate(([z1],[y1]), axis = 0)

def plot(z_max, y_max, init_angle, init_x, init_height):
	truth(z_max, y_max)
	plt.plot(init_x, init_height, 'r+')

	FOV = 57
	angle_min = init_angle-(FOV/2)
	angle_max = init_angle+(FOV/2)

	z_range = np.linspace(0,z_max,z_max*10+1)
	y_range = np.linspace(0,y_max,y_max*10+1)
	slope_min = m.tan(m.radians(angle_min))
	slope_max = m.tan(m.radians(angle_max))

	min_line = slope_min*z_range+init_height
	pos_min = min_line[min_line>=0]
	z_min = z_range[0:pos_min.size]
	plt.plot(z_min, pos_min)

	max_line = slope_max*z_range+init_height
	pos_max = max_line[max_line>=0]
	z_max = z_range[0:pos_max.size]
	plt.plot(z_max, pos_max)

	plt.plot(z_range, max_line)
	plt.show()
plot(20,10,init_angle, init_x, init_height)

def get_intersect(a1, a2, b1, b2):
    """ 
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])	       # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        pass

    x_range = np.arange(a1[0],a2[0],0.1)
    y_range = np.arange(a1[1], a2[1],0.1)
    if x_range.size == 0:
    	if np.around([y/z]) in y_range:
    		return np.array([x/z, y/z])
    if y_range.size == 0:
    	if np.around([x/z]) in x_range:
    		return np.array([x/z, y/z])
    elif np.around([x/z]) in x_range and np.around([y/z]) in y_range:
    	return np.array([x/z, y/z])
    else:
    	pass

def test_intersect():
	np.testing.assert_equal(get_intersect([0,0],[1,1],[0,1],[1,0]), [0.5, 0.5])
	assert get_intersect([0,0],[1,1],[2,2],[3,4]) == None
	#vertical line intersect
	np.testing.assert_equal(get_intersect([0,1],[4,1],[0,0],[5,5]), [1., 1.])

	print("Passed!")
test_intersect()

#makes a pointcloud of the truth terrain
# truth = truth()
# def pointcloud(init_angle, init_height, init_x, truth):
#     pointcloud = np.array([])
#     FOV = 57
#     angle_min = init_angle-(FOV/2)
#     angle_max = init_angle+(FOV/2)
#     origin = [init_x, init_height]
#     for angle in np.arange(angle_min, angle_max):
#         slope = m.tan(m.radians(angle))
#         ray_point = [init_x+1, (init_x+1)*slope]
#         # find the closest set of points to ray, a1, a2
#         intersect = get_intersect(a1,a2,origin,ray_point)
#         if intersect is not None:
#         	pointcloud = np.append(pointcloud,, axis=0)


#         # i = 0
#         # for i in range(np.size(truth,1)):
#         #     coordinate = truth[:,i]
#         #     z = round(coordinate[0],1)
#         #     y = round(coordinate[1],1)
#         #     if (pointIsOnLine(slope,0,z,y)):
#         #         pointcloud = np.append(pointcloud,[z, y], axis=0)
#         #         i+=1
#         #     else:
#         #         i+=1
#     return pointcloud

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


