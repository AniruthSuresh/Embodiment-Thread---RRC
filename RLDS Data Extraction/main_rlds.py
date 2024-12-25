import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import shutil


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

# robot_id = p.loadURDF ("./panda.urdf" ,[0, 0, 0], useFixedBase=True)
# robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Gunjan_Franka/model_description/panda.urdf", [0, 0, 0], useFixedBase=True)

robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Exact_Panda/bullet3/examples/pybullet/gym/pybullet_data/franka_panda/panda_with_2F85.urdf",[0, 0, 0], useFixedBase=True)


# robot_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
end_effector_link_index = 7
positions = []

# with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/data/scene_3/cart_pos.txt", "r") as file:
# # with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/cart_pos.txt", "r") as file:

#     for line in file:
#         positions.append(eval(line.strip()))

joint_positions = []

with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/RLDS_Data/scene_4/joint_ps.txt", "r") as file:
    for line in file:
        try:
            joint_positions.append(eval(line.strip()))  # Read joint positions as a list
        except SyntaxError as e:
            print(f"Syntax error in line: {line.strip()}")
            print(e)


# def move_to_position_with_feedback(target_position, target_orientation):

#     ik_joint_positions = p.calculateInverseKinematics(
#         robot_id, 
#         end_effector_link_index, 
#         target_position, 
#         target_orientation)
    

#     for i in range(len(ik_joint_positions)):
#         p.setJointMotorControl2(
#             bodyUniqueId=robot_id,
#             jointIndex=i,
#             controlMode=p.POSITION_CONTROL,
#             targetPosition=ik_joint_positions[i]
#         )


#     joint_angles = p.calculateInverseKinematics(robot_id, end_effector_link_index, target_position, targetOrientation=target_orientation , 
#                                              maxNumIterations=1000 )

#     for i in range(len(joint_angles)):
#         p.resetJointState(robot_id , i,joint_angles[i])


    # joint_positions = p.calculateInverseKinematics(
    #     robot_id,
    #     end_effector_link_index,
    #     target_position,
    #     target_orientation
    # )

    # # Ignore physics by directly resetting joint states
    # for i in range(len(joint_positions)):
    #     p.resetJointState(robot_id, i, joint_positions[i])


    # for _ in range(500): 
    #     p.stepSimulation()
    


def move_to_joint_position_with_feedback(joint_positions):
    # Use the joint positions directly and move the robot arm accordingly.
    for i in range(len(joint_positions)):
        p.resetJointState(robot_id, i, joint_positions[i])

    for _ in range(500):  # Step through the simulation to allow motion.
        p.stepSimulation()


def update_intrinsic_matrix(K, old_dims, new_dims):
    """
    Update the intrinsic matrix K based on new image dimensions.

    Parameters:
    K (np.array): Original 3x3 intrinsic matrix.
    old_dims (tuple): Original dimensions (height, width).
    new_dims (tuple): New dimensions (height, width).

    """
    old_height, old_width = old_dims
    new_height, new_width = new_dims

    scale_w = new_width / old_width
    scale_h = new_height / old_height

    K_updated = K.copy()
    K_updated[0, 0] *= scale_w  # Scale fx
    K_updated[1, 1] *= scale_h  # Scale fy
    K_updated[0, 2] *= scale_w  # Scale cx
    K_updated[1, 2] *= scale_h  # Scale cy

    return K_updated




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

    h = 180
    w = 320

    old_dims = (720 , 1280)
    new_dims = (180 , 320)


    """
    scene - 2
    """
    # K_old = np.array([
    #     [522.06170654, 0, 661.82672119],
    #     [0, 522.06170654, 355.39929199],
    #     [0, 0, 1]
    # ])

    """
    scene - 4
    """
    K_old = np.array([[524.12890625 , 0 , 639.77941895] , 
    [0,524.12890625 , 370.27819824] ,
    [0,0,1]] )


    K = update_intrinsic_matrix(K = K_old , old_dims = old_dims , new_dims = new_dims)

    print(K)


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




def capture_image(camera_position, camera_orientation, file_name):

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


    height = 180
    width = 320


    proj_matrix = cvK2BulletP()


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



filtered_image_dir = "filtered_arm_pics_fin_rlds"

if os.path.exists(filtered_image_dir):
    shutil.rmtree(filtered_image_dir)

os.makedirs(filtered_image_dir)


"""
scene - 2
"""
# the below one is for camera - 29431508 left 
# camera_position = [0.438195 ,0.504532 ,0.295449,]
# camera_orientation = p.getQuaternionFromEuler([-1.77092, -0.0652779, -2.76283])

# the below one is for camera - 29431508 right 
# camera_position = [0.326989,	0.459881	,0.303022]
# camera_orientation = p.getQuaternionFromEuler([-1.77039	,-0.0659621	,-2.76027])


"""
scene - 4
"""
camera_position = [0.085036	,0.563473	,0.416859]
camera_orientation = p.getQuaternionFromEuler([-1.95721,	-0.0233935	,-2.11812])
	


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
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 0.1])
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

draw_camera_direction(camera_position, camera_orientation)


# for idx, pos in enumerate(positions):
#     target_position = pos[:3]
#     target_orientation = p.getQuaternionFromEuler(pos[3:])

#     move_to_position_with_feedback(target_position, target_orientation)

#     image_name = os.path.join(filtered_image_dir, f"camera_position_{idx}.png")
#     capture_image(camera_position, camera_orientation, image_name)

for idx, joint_pos in enumerate(joint_positions):
    move_to_joint_position_with_feedback(joint_pos)  # Move to the joint positions directly

    # Capture and save an image of the current camera position
    image_name = os.path.join(filtered_image_dir, f"camera_position_{idx}.png")
    capture_image(camera_position, camera_orientation, image_name)



p.disconnect()


