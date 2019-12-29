import terrain_functions as t

p = t.initialize_camera()
try:
    for _ in range(5):
        f = p.wait_for_frames()
        position = t.position_data(f[0].as_pose_frame().get_pose_data().translation)
        roll_pitch_yaw = t.roll_pitch_yaw_calc(f[0].as_pose_frame().get_pose_data())

        #print(t.transformation_matrix_creator(position, roll_pitch_yaw))
        print(t.depth_vector_at_point(f,p,position,roll_pitch_yaw))

finally:
    p.stop()
