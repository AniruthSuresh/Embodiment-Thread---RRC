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

# p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)


robot_id = p.loadURDF("../../Exact_Panda/bullet3/examples/pybullet/gym/pybullet_data/franka_panda/panda_with_2F85_sec.urdf",[0, 0, 0], useFixedBase=True)

end_effector_link_index = 7
positions = []



joint_positions = []


with open("../Droid Mask Extraction/RLDS_Data/scene_17/joint_ps.txt", "r") as file:
    for line in file:
        try:
            position = [float(value) for value in line.strip().split()]
            joint_positions.append(position)  # Read joint positions as a list
        except SyntaxError as e:
            print(f"Syntax error in line: {line.strip()}")
            print(e)




def move_to_joint_position_with_feedback(joint_positions):
    # Use the joint positions directly and move the robot arm accordingly.
    for i in range(len(joint_positions)):
        p.resetJointState(robot_id, i, joint_positions[i])

    for _ in range(500):  # Step through the simulation to allow motion.
        p.stepSimulation()


def update_intrinsic_matrix(K, old_dims, new_dims):
    """
    Update the intrinsic matrix K based on new image dimensions.
    """
    
    # NOTE :  Mention the site later !

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
    NOTE : These are col - no 
    """

    """
    RPL setup 
    """
    # K_old = np.array([
    #     [522.06170654, 0, 661.82672119],
    #     [0, 522.06170654, 355.39929199],
    #     [0, 0, 1]
    # ])

    """
    scene - 12 
    """
    #     K_old = np.array([
    #     [522.845, 0, 648.825],
    #     [0, 522.845, 354.744],
    #     [0, 0, 1]
    # ])


    K_old = np.array([[524.11791992,   0.        , 639.77850342],
       [  0.        , 524.11791992, 370.27789307],
       [  0.        ,   0.        ,   1.        ]])

    """
    All AutoLab setup 
    """
    # K_old = np.array([[524.12890625 , 0 , 639.77941895] , 
    # [0,524.12890625 , 370.27819824] ,
    # [0,0,1]] )

    # K_old = np.array([[524.24609375,   0.        , 639.77758789],
    #    [  0.        , 524.24609375, 370.27789307],
    #    [  0.        ,   0.        ,   1.        ]])

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



def capture_image(camera_position, camera_orientation):
    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 1])

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=camera_target_position,
        cameraUpVector=[0, 0, 1]
    )

    height = 180
    width = 320

    proj_matrix = cvK2BulletP()

    _, _, rgb_img, _, seg_img = p.getCameraImage(
        width=width,           
        height=height,           
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL 
    )

    arm_object_ids = [0]
    seg_array = np.reshape(seg_img, (height, width))
    arm_mask = np.isin(seg_array, arm_object_ids).astype(np.uint8) * 255
    seg_mask = cv2.cvtColor(arm_mask, cv2.COLOR_GRAY2BGR)

    return rgb_img, seg_mask

def save_images(rgb_images, mask_images, start_num, rgb_folder="rgb_images_franka", mask_folder="mask_images_franka"):

    os.makedirs(rgb_folder, exist_ok=True)
    os.makedirs(mask_folder, exist_ok=True)

    for i, (rgb_img, mask_img) in enumerate(zip(rgb_images, mask_images), start=start_num):
        rgb_path = os.path.join(rgb_folder, f"image_{i}.png")
        mask_path = os.path.join(mask_folder, f"image_{i}.png")

        cv2.imwrite(rgb_path, rgb_img)
        cv2.imwrite(mask_path, mask_img)

    print(f"Saved {len(rgb_images)} images starting from index {start_num}")





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
scene - 3
	
"""
# camera_position = [0.358499	,0.436766,	0.228544]
# camera_orientation = p.getQuaternionFromEuler([-1.85161	,0.0533807	,-3.01635])


"""
scene - 4 - GOOD !
"""
# left
# camera_position = [0.085036	,0.563473	,0.416859]
# camera_orientation = p.getQuaternionFromEuler([-1.95721,	-0.0233935	,-2.11812])

#right
# camera_position = [0.221758	,-0.31568,	0.405043]
# camera_orientation = p.getQuaternionFromEuler([-2.04106,	0.0181328,	-0.47797])


"""
scene - 5
"""
# left 
# camera_position = [0.190077	,0.678763,	0.397614]
# camera_orientation = p.getQuaternionFromEuler([-2.00402,	0.0932804	,-2.29782])
	
	
# right 
# camera_position = [0.107339,	0.590132,	0.383264]
# camera_orientation = p.getQuaternionFromEuler([-2.00627,	0.0890864	,-2.28867])
	

"""
scene - 6
"""   

# 0.0315108	0.368786	0.436813	-2.12647	-0.138592	-1.66187
# camera_position = [0.0315108	,0.368786,	0.436813]
# camera_orientation = p.getQuaternionFromEuler([	-2.12647,-0.138592	,-1.66187])

"""
scene - 7
"""
# left cam -> good !
# camera_position = [0.167891,	0.447045,	0.488312]
# camera_orientation = p.getQuaternionFromEuler([-1.75215	,-0.0124033,-2.05865])


# right cam 0.236917	-0.298693	0.514985	-1.81304	0.00407764	-0.9203
# camera_position = [0.236917,	-0.298693,	0.514985]
# camera_orientation = p.getQuaternionFromEuler([-1.81304	,0.00407764,-0.9203])


"""
scene - 8 
	
"""
# camera_position = [0.421479	,0.689858,	0.510168]
# camera_orientation = p.getQuaternionFromEuler([-2.15771	,-0.150342	,-2.98978])

"""
scene -9 
	
"""
# camera_position = [0.225655	,0.60718	,0.286266]
# camera_orientation = p.getQuaternionFromEuler([-1.98394	,-0.0132214	,-2.42917])

"""
scene - 10 	
"""
# camera_position = [0.225655	,0.60718	,0.286266]
# camera_orientation = p.getQuaternionFromEuler([-1.98394,	-0.0132214	,-2.42917])

"""
scene - 11
	
"""
# camera_position = [0.421479	,0.689858,	0.510168]
# camera_orientation = p.getQuaternionFromEuler([-2.15771,	-0.150342,	-2.98978])

"""
Scene -12 : RPL 
	
"""
# camera_position = [0.0282164	,0.456743,	0.321209]
# camera_orientation = p.getQuaternionFromEuler([-1.37115	,0.00832112	,-1.82577 ])


# """
# scene 14 
# """
# camera_position = [-0.172387,	0.876985	,0.640223]
# camera_orientation = p.getQuaternionFromEuler([-2.0162	,-0.141334,	-2.15351 ])

	

"""
scene 15
"""
# camera_position = [0.131159	,0.545619,	0.385229]
# camera_orientation = p.getQuaternionFromEuler([-1.87873	,-0.021128,	-2.42463 ])


	

"""
scene 16
"""
# camera_position = [0.403975,	0.473188	,0.271706]
# camera_orientation = p.getQuaternionFromEuler([-1.68271	,0.0755023,	-2.66829 ])


"""
scene - 17
"""
camera_position = [0.285058	,0.529354,	0.505306]
camera_orientation = p.getQuaternionFromEuler([-2.15741	,-0.0145997	,-2.59569 ])


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


start_num = 894

rgb_images = []
mask_images = []

for idx, joint_pos in enumerate(joint_positions):
    move_to_joint_position_with_feedback(joint_pos)  # Move to the joint positions directly

    # Capture RGB and mask images
    rgb_img, mask_img = capture_image(camera_position, camera_orientation)
    rgb_images.append(rgb_img)
    mask_images.append(mask_img)

# Save images
save_images(rgb_images, mask_images, start_num)

p.disconnect()
