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

robot_id = p.loadURDF ("./panda.urdf" ,[0, 0, 0], useFixedBase=True)
# robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Gunjan_Franka/model_description/panda.urdf", [0, 0, 0], useFixedBase=True)


end_effector_link_index = 7
positions = []

# with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/data/scene_3/cart_pos.txt", "r") as file:
# # with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/cart_pos.txt", "r") as file:

#     for line in file:
#         positions.append(eval(line.strip()))

joint_positions = []
with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/data/scene_2/joint_pos.txt", "r") as file:
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

    h = 720
    w = 1280



    """
    GuptaLab
    """

    K = np.array([
        [522.49145508, 0, 653.96160889],
        [0, 522.49145508, 358.78604126],
        [0, 0, 1]
    ])


    """
    AutoLab
    """

    # K = np.array([
    # [524.12890625 , 0 , 639.77941895] , 
    # [0,524.12890625 , 370.27819824] ,
    # [0,0,1] 
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


    height = 720
    width = 1280


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



filtered_image_dir = "filtered_arm_pics_fin"

if os.path.exists(filtered_image_dir):
    shutil.rmtree(filtered_image_dir)

os.makedirs(filtered_image_dir)


# positions - 1
# camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
# camera_orientation = p.getQuaternionFromEuler([-1.5755163454721475, 0.009724033533159426, -2.1473608246676887])

# positions - 2
# camera_position = [ 0.13115858 , 0.54561889 , 0.38522926]
# camera_orientation = p.getQuaternionFromEuler([ -1.87873261, -0.02112802, -2.42462708])

camera_position = [ 0.10648816 , 0.74258522,  0.42642409]
camera_orientation = p.getQuaternionFromEuler([ -1.62486946, -0.0689483,  -2.59861434])

# camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
# camera_orientation = p.getQuaternionFromEuler([-1.5755163454721475, 0.009724033533159426, -2.1473608246676887])

# camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
# camera_orientation = p.getQuaternionFromEuler([-1.5755163454721475, 0.009724033533159426, -2.1473608246676887])

# [0.0625700860308161, 0.7849934078474259, 0.41321386028430906, -1.620310649739649, 0.0378335771770526, -2.5025320852316324]


# camera_position = [0.0625700860308161, 0.7849934078474259, 0.41321386028430906]
# camera_orientation = p.getQuaternionFromEuler([-1.620310649739649, 0.0378335771770526, -2.5025320852316324])


# camera_position = [0.2256552520414773 , 0.6071800031562721 , 0.28626581597666506]
# camera_orientation = p.getQuaternionFromEuler([-1.9839358570154344 , -0.013221358635326697 , -2.4291658876333457])



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


