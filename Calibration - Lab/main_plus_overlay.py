import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

import pandas as pd
import re
import shutil

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)
    
# robot_id = p.loadURDF ("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/panda.urdf" ,[0, 0, 0], useFixedBase=True)
robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/xarm7_robot.urdf", [0, 0,0], useFixedBase=True)

"""
The robot base is placed at origin (since the base_link is at a certain offset) in the below case
"""

# robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/xarm7_robot.urdf", [0.021131, 0.0016302, -0.056488], useFixedBase=True)

# -0.021131, -0.0016302, 0.056488 -> link_base position 


# In xarm - 7 , the first link is considered as base , so always pass i+1 in IK or joint state settings !

# num_joints = p.getNumJoints(robot_id)
# joint_indices = {}
# print(f"No of joints : {num_joints}")
# for i in range(num_joints):
#     joint_info = p.getJointInfo(robot_id, i)
#     joint_name = joint_info[1].decode('utf-8')
#     joint_indices[joint_name] = i
#     link_name = joint_info[12].decode('utf-8')
#     print(f"Joint {i}: {joint_name} controls Link {link_name}")

# print("Joint Indices:", joint_indices)


end_effector_link_index = 7

positions = []

csv_file_path = 'cleaned_js.csv'  # Update with your actual file path
data = pd.read_csv(csv_file_path)


joint_states_array = np.array(data['joint_states'].str.split(', ').tolist(), dtype=float)

print(len(joint_states_array))

def move_to_joint_angles(joint_angles):

    current_joint_states = [p.getJointState(robot_id, i)[0] for i in range(len(joint_angles))]
    print(f"Current Joint States: {current_joint_states}")

    for i in range(len(joint_angles)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i+1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angles[i]
        )

    for _ in range(500): 
        p.stepSimulation()
    
    actual_joint_states = [p.getJointState(robot_id, i)[0] for i in range(len(joint_angles))]
    # print(f"Actual Joint States: {actual_joint_states}")

def cvK2BulletP():
    """
    cvKtoPulletP converst the K interinsic matrix as calibrated using Opencv
    and ROS to the projection matrix used in openGL and Pybullet.

    :param K:  OpenCV 3x3 camera intrinsic matrix
    :param w:  Image width
    :param h:  Image height
    :near:     The nearest objects to be included in the render
    :far:      The furthest objects to be included in the render
    :return:   4x4 projection matrix as used in openGL and pybullet

    # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12901
    """ 

    near = 0.1
    far = 3.1

    """
    scene 1 
    """
    w = 640 
    h = 480

    """
    scene 2 
    """
    
    # w = 1280 
    # h = 720 

    K_0 = 386.225341796875
    K_1 = 0.0
    K_2 = 329.5455322265625
    K_4 = 385.2564392089844
    K_5 = 244.97164916992188

    """
    Scene 1 
    """

    K = np.array([
        [K_0, K_1, K_2],
        [0, K_4, K_5],
        [0, 0, 1]
    ])

    """
    Scene 2 
    """

    # K = np.array([
    #     [645.283203125, 0.0, 655.9091796875],
    #     [0.0, 643.66455078125, 368.2861022949219],
    #     [0.0, 0.0, 1.0]
    # ])




    f_x = K[0,0]
    f_y = K[1,1]
    c_x = K[0,2]
    c_y = K[1,2]

    A = (near + far)/(near - far)
    B = 2 * near * far / (near - far)

    projection_matrix = [
                        [2/w * f_x,  0,          (w - 2*c_x)/w,  0],
                        [0,          2/h * f_y,  (2*c_y - h)/h,  0],
                        [0,          0,          A,              B],
                        [0,          0,          -1,             0]]

    return np.array(projection_matrix).T.reshape(16).tolist()


def capture_and_filter_arm(camera_position, camera_orientation, file_name, arm_object_ids=None):
    if os.path.exists(file_name):
        os.remove(file_name)  

    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 1])

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=camera_target_position,
        cameraUpVector=[0, 0, 1]
    )

    """
    # https://reachpranjal19.medium.com/camera-calibration-in-ros-melodic-a0bf4742d636
    # https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
    """

    # proj_matrix = p.computeProjectionMatrixFOV(
    #     fov=70,          
    #     aspect= 640/480,     
    #     nearVal=0.1,     
    #     farVal=3.1       
    # )


    proj_matrix = cvK2BulletP()

    """
    Scene 1
    """
    width = 640 
    height = 480

    """
    Scene 2
    """
    # width = 1280 
    # height = 720 

    
    _, _, _, _, seg_img = p.getCameraImage(
        width=width,           
        height=height,           
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL 
    )

    arm_object_ids = [1]


    seg_array = np.reshape(seg_img, (height, width))
    arm_mask = np.isin(seg_array, arm_object_ids).astype(np.uint8) * 255
    seg_mask = cv2.cvtColor(arm_mask, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(file_name, seg_mask)

additional_image_dir = "seg_mask_images"
os.makedirs(additional_image_dir, exist_ok=True)



# METHOD 1 : STANDARD 

# camera_position = [0.701274, 1.0029, -0.00991212]
# camera_orientation_place = [2.97629, -3.13523, 1.31308]
# camera_orientation = [-0.0478799, -0.0673292, -0.789239, 0.608503] # in quat 


# camera_orientation_euler = [2.97629, -3.13523, 1.31308]  # Euler angles in radians
# camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)

# camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
# camera_orientation = [-0.0478799 ,-0.0673292 ,-0.789239 ,0.608503]


# METHOD 2 : INVERSE

# inverse_translation =  [ 1.13711832 ,-0.42651768 ,-0.15078622]
# inverse_rotation = [0.16530294, 0.00636272, 1.82850777]
# camera_position = inverse_translation
# camera_orientation = p.getQuaternionFromEuler(inverse_rotation)


# METHOD 3 : Direct transformation from the tree 

# camera_position = [0.644 , 1.018 , -0.013]
# camera_orientation_euler = [-1.408 , 0.045 , 2.889]  # Euler angles in radians
# camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)

# METHOD 4 : From link -> camera_link and then camera_link -> camera_colour_optical_frame
"""
Doesnt consider the base drift transformation !
"""
camera_position =  [0.64422358 , 1.01767251, -0.01274457]
camera_orientation_euler = [-1.40861673 , 0.04592545,  2.88869117]
camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)

# METHOD 5 :  Includes drift : From base -> link_base -> camera_link and then camera_link -> camera_colour_optical_frame


# Translation from base to camera_optical_colour_frame: []
# RPY from base to camera_optical_colour_frame (in radians): [-1.56068014 -0.16730432  2.88360939]

"""
This is the final one for now considering the base drift transformation !
"""
# camera_position =  [  0.66614694 , 1.01942245 ,-0.05669182]
# camera_orientation_euler =  [-1.56068014 ,-0.16730432 , 2.88360939]
# camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)

# Translation from base to camera_optical_colour_frame: [0.62388494 1.01616205 0.05628418]
# RPY from base to camera_optical_colour_frame (in radians): [-1.56068014 -0.16730432  2.88360939]



"""
SECOND SCENE : Without considering the base drift 
"""

# camera_position =  [1.12790676 ,0.44972272, 0.19800316]
# camera_orientation_euler =  [-1.63617699, -0.02267317,  1.97343222]
# camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)


"""
SECOND SCENE : Considering the base drift 
"""
# camera_position =  [1.14903776, 0.45135292, 0.14151516]
# camera_orientation_euler =  [-1.63617699 ,-0.02267317 , 1.97343222]
# camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)

"""
Accounting for the quat here 
[1.12803471 0.44978872 0.19723897] [-1.49523336 -0.00983111  1.97071679]

"""
# camera_position =  [1.12790676 ,0.44972272, 0.19800316]
# camera_orientation_euler =  [-1.49523336 ,-0.00983111 , 1.97071679]
# camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)



def draw_camera_direction(camera_position, camera_orientation):

    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 2])
    print("hello")
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

draw_camera_direction(camera_position, camera_orientation)



visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.05, 0.05, 0.05],
    rgbaColor=[0, 1, 0, 1]  
)

camera_body_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=camera_position,
    baseOrientation=camera_orientation
)


def draw_camera_direction(camera_position, camera_orientation):
    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 2])
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

# draw_camera_direction(camera_position, camera_orientation)

def extract_number(filename):
    """
    Extracts the first number found in the filename
    """

    numbers = re.findall(r'\d+', filename)
    return int(numbers[0]) if numbers else float('inf')


def overlay_images(folder1, folder2, output_folder, alpha=0.5):

    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)  # Remove the entire output directory
    os.makedirs(output_folder)  

    image_filenames1 = sorted(os.listdir(folder1), key=extract_number)
    image_filenames2 = sorted(os.listdir(folder2), key=extract_number)


    if len(image_filenames1) != len(image_filenames2):
        print("Error: The number of images in both folders does not match.")
        return

    for index in range(len(image_filenames1)):
        img1_path = os.path.join(folder1, image_filenames1[index])
        img2_path = os.path.join(folder2, image_filenames2[index])

        print(img1_path)
        # Read the images
        img1 = cv2.imread(img1_path)
        img2 = cv2.imread(img2_path)

        if img1 is not None and img2 is not None:
            img2 = cv2.resize(img2, (img1.shape[1], img1.shape[0]))

            blended = cv2.addWeighted(img1, 1 - alpha, img2, alpha, 0)

            output_path = os.path.join(output_folder, f"overlay_{index}.png")
            cv2.imwrite(output_path, blended)

            print(f"Saved overlay image: {output_path}")
        else:
            print(f"Could not load images: {img1_path} or {img2_path}")


def frames_to_video(frames_folder, output_video, fps):
    frame_files = [f for f in os.listdir(frames_folder) if f.endswith('.png')]
    frame_files.sort(key=lambda x: int(re.search(r'(\d+)', x).group()))

    if not frame_files:
        print("No frame files found in the folder.")
        return
    
    first_frame_path = os.path.join(frames_folder, frame_files[0])
    frame = cv2.imread(first_frame_path)
    height, width, layers = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    # Write each frame to the video
    for frame_file in frame_files:
        frame_path = os.path.join(frames_folder, frame_file)
        img = cv2.imread(frame_path)
        video.write(img)

    video.release()
    print(f"Video saved as {output_video}")



for idx, joint_angles in enumerate(joint_states_array):

    # print(f"Joint angle to {joint_angles}")

    move_to_joint_angles(joint_angles)
    image_name = os.path.join(additional_image_dir, f"camera_position_{idx}.png")
    capture_and_filter_arm(camera_position, camera_orientation, image_name)
    draw_camera_direction(camera_position, camera_orientation)



folder1 ="cleaned_image"  
folder2 = "seg_mask_images" 
output_folder = 'final_result'  

overlay_images(folder1, folder2, output_folder, alpha=0.5)

frames_to_video(output_folder , output_video = 'scene_1_quat.mp4' , fps= 50)

p.disconnect()




