import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

import pandas as pd


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

# robot_id = p.loadURDF ("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/panda.urdf" ,[0, 0, 0], useFixedBase=True)
robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/xarm7_robot.urdf", [0, 0, 0], useFixedBase=True)

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

def projection_using_intrinsics():
    """
    Computes the proj matrix using the intrinsics obtained during calib
    """

    P_values = {
        'P_0': 386.225341796875,
        'P_1': 0.0,
        'P_2': 329.5455322265625,
        'P_3': 0.0,
        'P_4': 0.0,
        'P_5': 385.2564392089844,
        'P_6': 244.97164916992188,
        'P_7': 0.0,
        'P_8': 0.0,
        'P_9': 0.0,
        'P_10': 1.0,
        'P_11': 0.0,
    }

    projection_matrix = np.array([
        [P_values['P_0'], P_values['P_1'], P_values['P_2'], P_values['P_3']],
        [P_values['P_4'], P_values['P_5'], P_values['P_6'], P_values['P_7']],
        [P_values['P_8'], P_values['P_9'], P_values['P_10'], P_values['P_11']],
        [0, 0, 0, 1]
    ])


    return projection_matrix.flatten()




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

    proj_matrix = p.computeProjectionMatrixFOV(
        fov=65,          
        aspect= 640/480,     
        nearVal=0.1,     
        farVal=3.1       
    )


    # print(proj_matrix)

    # proj_matrix = projection_using_intrinsics()
    # print(proj_matrix.shape)

    # adjusted for lab calibration -> the height and width 
    width, height, img, _, _ = p.getCameraImage(
        width=640,           
        height=480,           
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL 
    )

    rgb_array = np.array(img)
    rgb_array = rgb_array[:, :, :3]  # No alpha chan

    bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)

    # cv2.imwrite(file_name, bgr_array)

additional_image_dir = "rgb_images"
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

camera_position =  [ 6.45015938e-01  , 1.01779225e+00 ,-2.03815391e-04]
camera_orientation_euler = [-1.56068014 , -0.16730432 , 2.88360939]
camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)



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

draw_camera_direction(camera_position, camera_orientation)



for idx, joint_angles in enumerate(joint_states_array):

    # print(f"Joint angle to {joint_angles}")

    move_to_joint_angles(joint_angles)
    image_name = os.path.join(additional_image_dir, f"camera_position_{idx}.png")
    capture_and_filter_arm(camera_position, camera_orientation, image_name)



p.disconnect()

