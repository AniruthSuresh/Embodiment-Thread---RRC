import airobot
import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
from airobot.sensor.camera.rgbdcam_pybullet import RGBDCameraPybullet
from scipy.spatial.transform import Rotation as R
import time

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

    return robot



def visualize_joint_axes(robot):

    # Visualize the base of the robot
    base_pos, base_orient = p.getBasePositionAndOrientation(robot.arm.robot_id)
    base_rot_matrix = R.from_quat(base_orient).as_matrix()

    # Draw X, Y, Z axes of the base
    axes_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # Colors for X, Y, Z
    # for i, color in enumerate(axes_colors):
    #     axis_endpoint = base_pos + base_rot_matrix[:, i] * 0.2  # Scale for visibility
    #     p.addUserDebugLine(base_pos, axis_endpoint, lineColorRGB=color, lineWidth=2)

    # Iterate over the joints
    for joint_id in range(robot.arm.arm_dof):

            joint_pos, joint_orient = p.getLinkState(robot.arm.robot_id, joint_id)[0:2]
            rot_matrix = R.from_quat(joint_orient).as_matrix()

            # Draw X, Y, Z axes for each joint
            for i, color in enumerate(axes_colors):
                axis_endpoint = joint_pos + rot_matrix[:, i] * 0.2  # Scale for visibility
                p.addUserDebugLine(joint_pos, axis_endpoint, lineColorRGB=color, lineWidth=2)

def main():
    gui = True
    check_realtime = True
    robot = create_robot(gui, check_realtime)

    print("Current End Effector Pose:", robot.arm.get_ee_pose())

    visualize_joint_axes(robot)

    print("Axes visualization complete. Close the GUI to exit.")
    while p.isConnected():
        time.sleep(1)

if __name__ == "__main__":
    main()
