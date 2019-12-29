import pyrealsense2 as rs
import numpy as np

try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()
    pipeline.start()

    depth_sensor = pipeline.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    for _ in range(1):
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth: continue

        data = np.array([])
        for y in range(480):
            x = 320
            dist = depth.get_distance(x, y)*depth_scale
            data = np.append(data, dist)
        print(data)

finally:
    pipeline.stop()