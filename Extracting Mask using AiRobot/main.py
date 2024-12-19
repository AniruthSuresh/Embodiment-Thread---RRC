import airobot
import pybullet as p
import numpy as np
from airobot.sensor.camera.rgbdcam_pybullet import RGBDCameraPybullet
import cv2 
import os
import shutil
import time


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
                self.FOV = 65


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


def move_to_position(robot, target_position, target_orientation):
    """
    Move the robot to a specified target position and orientation in multiple smaller steps.
    """
    target_joint_positions = robot.arm.compute_ik(target_position, target_orientation)
    current_joint_positions = robot.arm.get_jpos()
    robot.arm.set_jpos(target_joint_positions, wait=False, ignore_physics=True)


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


def setup_camera(robot, position, orientation):
    """
    Setup the camera at a specified position and orientation using set_cam_ext.
    """

    position_array = np.array(position) 
    robot.cam.set_cam_ext(pos=position_array, ori=orientation)  
    print(f"Camera set to position: {position} with orientation: {orientation}")


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


def place_robot_on_ground(robot , ground_plane_id):
    """
    Gradually lower the robot to the ground by checking for collisions with the ground plane.
    """
    while pairwise_collision(robot.arm.robot_id,ground_plane_id)==False:  

        print("here")
        current_position, current_orientation = p.getBasePositionAndOrientation(robot.arm.robot_id)
        new_position = list(current_position)
        new_position[2] -= 0.2 
        if(new_position[2] < 0 ):
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
    camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
    camera_orientation = p.getQuaternionFromEuler([-1.5755163454721475, 0.009724033533159426, -2.1473608246676887])
    
    # Camera position and orientation - scene 2
    # camera_position = [ 0.13115858 , 0.54561889 , 0.38522926]
    # camera_orientation = p.getQuaternionFromEuler([ -1.87873261, -0.02112802, -2.42462708])


    setup_camera(robot, camera_position, camera_orientation)
    directory_name = 'images'

    if os.path.exists(directory_name):
        shutil.rmtree(directory_name)
        print(f"Deleted existing directory: {directory_name}")

    os.makedirs(directory_name, exist_ok=True)
    print(f"Created directory: {directory_name}")

    positions = []
    # Extracting the end eff pos and ori from the text file
    with open("../airobot_rtvs/position.txt", "r") as file:
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


if __name__ == "__main__":
    main()

