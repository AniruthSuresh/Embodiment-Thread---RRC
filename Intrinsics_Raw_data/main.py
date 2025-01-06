import os
from camera_utils import SVOReader
import cv2

svo_files = ["/home/aniruth/Desktop/RRC/Embodiment-Thread---RRC/Intrinsics_Raw_data/data/22246076.svo"]

for svo_file in svo_files:
    serial_number = svo_file.split("/")[-1][:-4]
    print(f"Processing SVO file: {serial_number}")

    # Create folders for left and right images
    left_folder = os.path.join(os.getcwd(), "images_left")
    right_folder = os.path.join(os.getcwd(), "images_right")
    os.makedirs(left_folder, exist_ok=True)
    os.makedirs(right_folder, exist_ok=True)

    reader = SVOReader(svo_file, serial_number)

    reader.set_reading_parameters(image=True, concatenate_images=False)

    camera_intrinsics = reader.get_camera_intrinsics()
    camera_baseline = reader.get_camera_baseline()

    print(f"Camera Intrinsics (Left): {camera_intrinsics[serial_number + '_left']}")
    print(f"Camera Intrinsics (Right): {camera_intrinsics[serial_number + '_right']}")
    print(f"Camera Baseline: {camera_baseline} meters")

    frame_count = reader.get_frame_count()
    print(f"Total frames in the SVO file: {frame_count}")

    for frame_index in range(frame_count):
        reader.set_frame_index(frame_index)

        data = reader.read_camera(ignore_data=False)
        left_image = data["image"].get(serial_number + "_left")
        right_image = data["image"].get(serial_number + "_right")

        if left_image is not None and right_image is not None:
            # Save left and right images into their respective folders
            left_image_path = os.path.join(left_folder, f"{serial_number}_left_{frame_index:04d}.png")
            right_image_path = os.path.join(right_folder, f"{serial_number}_right_{frame_index:04d}.png")
            cv2.imwrite(left_image_path, left_image)
            cv2.imwrite(right_image_path, right_image)

    print("Finished extracting images and parameters.")
