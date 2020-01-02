import CC_terrain_functions as t
import pyrealsense2 as rs

import sys # computer-specific module 
import os # operating system module (path management)
# add subfolder into default path of folders
sys.path.append(os.getcwd() + '/helper_functions')
# now module can be imported from subfolder
from realsense_device_manager import DeviceManager

# p = rs.pipeline()
# conf = rs.config()
# conf.enable_stream(rs.stream.pose)
# conf.enable_stream(rs.stream.depth)

# device_manager = DeviceManager(rs.context(), conf)
# device_manager.enable_all_devices()

# prof = p.start(conf)

p = t.initialize_camera()

try:
    for _ in range(50):
        f = p.wait_for_frames()
        position = t.position_data(f[0].as_pose_frame().get_pose_data().translation)
        roll_pitch_yaw = t.roll_pitch_yaw_calc(f[0].as_pose_frame().get_pose_data())
        # depth = f.get_depth_frame().get_data()
        print(t.transformation_matrix_creator(position, roll_pitch_yaw))
        # print(depth.get_distance(0, 0))
        # if not depth: continue
        # print(t.project_point(f,depth,position,roll_pitch_yaw))

finally:
    p.stop()