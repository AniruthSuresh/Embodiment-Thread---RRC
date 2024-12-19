import airobot
import pybullet as p
import numpy as np
from airobot.sensor.camera.rgbdcam_pybullet import RGBDCameraPybullet
import cv2 
import os
import shutil
import time
from scipy.spatial.transform import Rotation as R


class CameraConfig:
    """Class to define camera configuration structure."""
    def __init__(self):
        self.CAM = self.CameraSettings()

    class CameraSettings:
        """Nested class to define camera settings."""
        def __init__(self):
            self.SIM = self.SimulationSettings()

        class SimulationSettings:
            """Nested class to define simulation settings."""
            def __init__(self):
                self.ZNEAR = 0.1 
                self.ZFAR = 3.1  
                self.WIDTH = 1280 
                self.HEIGHT = 720
                self.FOV = 55


def create_robot(gui: bool, check_realtime: bool, check_render: bool = False):
    """
    Create the Franka robot (airobot - not custom)
    """
    robot = airobot.Robot(
        "franka",  
        pb_cfg={"gui": gui, "realtime": check_realtime, "opengl_render": check_render},
        use_cam=True
    )
    
    success = robot.arm.go_home(ignore_physics=True)
    
    if not success: 
        print("ehhhhh!")

    assert p.isConnected(robot.pb_client.get_client_id())

    camera_config = CameraConfig()
    
    robot.cam = RGBDCameraPybullet(cfgs=camera_config, pb_client=robot.pb_client) 
    return robot


# def move_to_position(robot, target_position, target_orientation):
#     """
#     Move the robot to a specified target position and orientation in multiple smaller steps.
#     """
#     target_joint_positions = robot.arm.compute_ik(target_position, target_orientation)
#     current_joint_positions = robot.arm.get_jpos()
#     robot.arm.set_jpos(target_joint_positions, wait=False, ignore_physics=True)


def move_to_position(robot, target_position, target_orientation):
    """
    Move the robot to a specified target position and orientation 
    Adjusts for the gripper offset to ensure the actual gripper end reaches the target.
    """
    gripper_offset = np.array([0, 0, -0.1])  

    rot_matrix = R.from_quat(target_orientation).as_matrix()

    world_offset = rot_matrix @ gripper_offset

    adjusted_position = np.array(target_position) - world_offset

    target_joint_positions = robot.arm.compute_ik(adjusted_position, target_orientation)

    robot.arm.set_jpos(target_joint_positions, wait=False, ignore_physics=True)

    print(f"Adjusted Position: {adjusted_position}, Target Orientation: {target_orientation}")

    # place_a_ball(target_position, target_orientation, color=[1, 0, 0, 1])  # Red ball before adjustment
    # place_a_ball(adjusted_position, target_orientation, color=[0, 1, 0, 1])


def place_a_ball(position, orientation, radius=0.03):
    """
    Place a ball that ignores physics (no gravity, no forces applied) at the specified position.
    """
    collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[1, 0, 0, 1])

    ball_id = p.createMultiBody(baseMass=0.0,             
                                baseCollisionShapeIndex=collision_shape_id,
                                baseVisualShapeIndex=visual_shape_id,
                                basePosition=position,
                                baseOrientation=orientation)

    p.changeDynamics(ball_id, -1, mass=0, lateralFriction=0, spinningFriction=0, 
                     rollingFriction=0, linearDamping=0, angularDamping=0)
    
    p.setGravity(0, 0, 0) 
    p.changeDynamics(ball_id, -1, activationState=p.ACTIVATION_STATE_DISABLE_SLEEPING)  

    print(f"Ball placed at position: {position} with radius: {radius}")


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


def setup_camera(robot, position, orientation):
    """
    Setup the camera at a specified position, orientation, and target position using set_cam_ext.
    Computes the view matrix and projection matrix, and applies them to the camera.
    """

    position_array = np.array(position) 
    robot.cam.set_cam_ext(pos=position_array, ori=orientation)  
    print(f"Camera set to position: {position} with orientation: {orientation}")

    rot_matrix = R.from_quat(orientation).as_matrix()
    camera_target_position = position + rot_matrix @ np.array([0, 0, 1])

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=position, 
        cameraTargetPosition=camera_target_position,
        cameraUpVector=[0, 0, 1] 
    )
    
    projection_matrix = np.array(cvK2BulletP())
    view_matrix = np.array(view_matrix)

    """
    Reference : https://airobot.readthedocs.io/en/latest/airobot/sensor/camera/airobot.sensor.camera.rgbdcam_pybullet.html
    """
    robot.cam.view_matrix = view_matrix  
    robot.cam.proj_matrix = projection_matrix 
    
    print(f"View matrix set: {view_matrix}")
    print(f"Projection matrix set: {projection_matrix}")


def capture_and_save_image(robot, image_count):
    """
    Capture an RGB image from the camera and save it to the 'images' folder.
    """
    rgb_image, _, _ = robot.cam.get_images(
            True, True, True, shadow=0, lightDirection=[0, 0, 2]
        )    
    
    image_path = f"images/image_{image_count}.png"  
    cv2.imwrite(image_path, rgb_image)  
    print(f"Saved image: {image_path}")


def capture_and_save_image_mask(robot, image_count):
    """
    Capture an RGB image from the camera and save it to the 'images' folder.
    """
    _, _, seg_image = robot.cam.get_images(
            True, True, True, shadow=0, lightDirection=[0, 0, 2]
        )    
    
    image_path = f"images/image_{image_count}.png"  
    cv2.imwrite(image_path, seg_image)  
    print(f"Saved image: {image_path}")


def pairwise_collision(body_a_id, body_b_id):
    """
    Check if two objects (body_a and body_b) are in contact (colliding).
    Returns True if they are colliding, otherwise False.
    """
    contact_points = p.getContactPoints(bodyA=body_a_id, bodyB=body_b_id)
    return len(contact_points) > 0

def place_a_ball(position, orientation, radius=0.03, color=[1, 0, 0, 1]):
    """
    Place a ball at the specified position with a given color.
    Ball ignores physics (no gravity, no forces applied).
    """
    collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)

    ball_id = p.createMultiBody(baseMass=0.0,             
                                baseCollisionShapeIndex=collision_shape_id,
                                baseVisualShapeIndex=visual_shape_id,
                                basePosition=position,
                                baseOrientation=orientation)

    p.changeDynamics(ball_id, -1, mass=0, lateralFriction=0, spinningFriction=0, 
                     rollingFriction=0, linearDamping=0, angularDamping=0)
    
    p.setGravity(0, 0, 0) 
    p.changeDynamics(ball_id, -1, activationState=p.ACTIVATION_STATE_DISABLE_SLEEPING)  

    print(f"Ball placed at position: {position} with radius: {radius} and color: {color}")


def place_robot_on_ground(robot , ground_plane_id):
    """
    Gradually lower the robot to the ground by checking for collisions with the ground plane.
    """
    while pairwise_collision(robot.arm.robot_id,ground_plane_id)==False:  

        print("here")
        current_position, current_orientation = p.getBasePositionAndOrientation(robot.arm.robot_id)
        new_position = list(current_position)
        new_position[2] -= 0.01
 
        if(new_position[2] < 0.0):
            print("breaking because falling below ground")
            break
        p.resetBasePositionAndOrientation(robot.arm.robot_id, new_position, current_orientation)
        print(f"Lowering robot base to {new_position}")


        time.sleep(0.1) 

    print("Robot is now placed on the ground.")


def main():
    gui = True
    check_realtime = True 
    robot = create_robot(gui, check_realtime)
    print("Current End Effector Pose:", robot.arm.get_ee_pose())
    ground_plane_id = p.loadURDF("plane.urdf")

    place_robot_on_ground(robot , ground_plane_id)

    # # # Camera position and orientation - scene 1 
    # camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
    # camera_orientation = p.getQuaternionFromEuler([-1.5755163454721475, 0.009724033533159426, -2.1473608246676887])
    
    # # # Camera position and orientation - scene 2
    camera_position = [ 0.13115858 , 0.54561889 , 0.38522926]
    camera_orientation = p.getQuaternionFromEuler([ -1.87873261, -0.02112802, -2.42462708])
# 

    # camera_position = [0.10648816  ,0.74258522  ,0.42642409]
    # camera_orientation = p.getQuaternionFromEuler([-1.62486946, -0.0689483,  -2.59861434])
        
    # camera_position = [0.0625700860308161, 0.7849934078474259, 0.41321386028430906]
    # camera_orientation = p.getQuaternionFromEuler([-1.620310649739649, 0.0378335771770526, -2.5025320852316324])



    setup_camera(robot, camera_position, camera_orientation)
    directory_name = 'images'

    if os.path.exists(directory_name):
        shutil.rmtree(directory_name)
        print(f"Deleted existing directory: {directory_name}")

    os.makedirs(directory_name, exist_ok=True)
    print(f"Created directory: {directory_name}")

    positions = []
    # Extracting the end eff pos and ori from the text file
    with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/data/scene_1/cart_pos.txt", "r") as file:
        for line in file:
            try:
                positions.append(eval(line.strip()))
            except SyntaxError as e:
                print(f"Syntax error in line: {line.strip()}")
                print(e)


    image_count = 0 
    for idx, pos in enumerate(positions):
        target_position = list(pos[:3])
        target_orientation = p.getQuaternionFromEuler(pos[3:])

        # place_a_ball(target_position, target_orientation)
        print("Target Position:", target_position, "Target Orientation:", target_orientation)
        
        move_to_position(robot, target_position, target_orientation)
        capture_and_save_image_mask(robot, image_count)
        image_count += 1  


# def main():
#     gui = True
#     check_realtime = True 
#     robot = create_robot(gui, check_realtime)
#     print("Current End Effector Pose:", robot.arm.get_ee_pose())
#     ground_plane_id = p.loadURDF("plane.urdf")

#     place_robot_on_ground(robot , ground_plane_id)

#     # # # Camera position and orientation - scene 1 
#     # camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
#     # camera_orientation = p.getQuaternionFromEuler([-1.5755163454721475, 0.009724033533159426, -2.1473608246676887])
    
#     # # # Camera position and orientation - scene 2
#     camera_position = [ 0.13115858 , 0.54561889 , 0.38522926]
#     camera_orientation = p.getQuaternionFromEuler([ -1.87873261, -0.02112802, -2.42462708])


#     # camera_position = [0.10648816  ,0.74258522  ,0.42642409]
#     # camera_orientation = p.getQuaternionFromEuler([-1.62486946, -0.0689483,  -2.59861434])
        
#     # camera_position = [0.0625700860308161, 0.7849934078474259, 0.41321386028430906]
#     # camera_orientation = p.getQuaternionFromEuler([-1.620310649739649, 0.0378335771770526, -2.5025320852316324])


#     # camera_position = [0.2256552520414773 , 0.6071800031562721 , 0.28626581597666506]
#     # camera_orientation = p.getQuaternionFromEuler([-1.9839358570154344 , -0.013221358635326697 , -2.4291658876333457])



#     setup_camera(robot, camera_position, camera_orientation)
#     directory_name = 'images'

#     # Read joint positions from the text file
#     joint_positions = []
#     with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/data/scene_1/joint_pos.txt", "r") as file:
#         for line in file:
#             try:
#                 joint_positions.append(eval(line.strip()))  # Read joint positions as a list
#             except SyntaxError as e:
#                 print(f"Syntax error in line: {line.strip()}")
#                 print(e)

#     # Iterate through joint positions and move the robot
#     image_count = 0 
#     for idx, joint_pos in enumerate(joint_positions):
#         print(f"Target Joint Positions: {joint_pos}")

#         # Move the robot to the joint positions
#         robot.arm.set_jpos(joint_pos, wait=False, ignore_physics=True)

#         capture_and_save_image_mask(robot, image_count)
#         image_count += 1  




if __name__ == "__main__":
    main()

