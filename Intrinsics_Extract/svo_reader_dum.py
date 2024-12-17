from svo_reader import SVOReader


svo_files = ["/data/22246076.svo"]

cam_matrices = []
cam_baselines = []

for svo_file in svo_files:
    # Open SVO Reader
    serial_number = svo_file.split("/")[-1][:-4]
    print(serial_number)
    camera = SVOReader(svo_file, serial_number=serial_number)
    # camera.set_reading_parameters(image=True, pointcloud=False, concatenate_images=False)
    im_key = '%s_left' % serial_number
    # Intrinsics are the same for the left and the right camera
    cam_matrices.append(camera.get_camera_intrinsics()[im_key]['cameraMatrix'])
    cam_baselines.append(camera.get_camera_baseline())

    print(cam_matrices)



