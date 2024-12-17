import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane and robot
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)
# robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Gunjan_Franka/model_description/panda.urdf", [0, 0, 0], useFixedBase=True)
robot_id = p.loadURDF ("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/panda.urdf" ,[0, 0, 0], useFixedBase=True)
end_effector_link_index = 7

positions = []

with open("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/position.txt", "r") as file:
    for line in file:
        positions.append(eval(line.strip()))

position_differences = []


def plot_positions(positions):
    marker_radius = 0.02
    marker_color = [1, 0, 0, 1] 
    
    for pos in positions:
        position = pos[:3]
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=marker_radius,
            rgbaColor=marker_color
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=position
        )

plot_positions(positions)  

# iniial_pos = [0.301166, -0.105952, 0.5963394]
# initial_orientation = p.getQuaternionFromEuler([2.793354, -0.412777, 0.03653])  # Roll, pitch, yaw to quaternion

# joint_angles = p.calculateInverseKinematics(robot_id, end_effector_link_index, iniial_pos, targetOrientation=initial_orientation , 
#                                              maxNumIterations=1000  )
# print(f"Calculated Joint Positions: {joint_angles}")

# # print()
# for i in range(len(joint_angles)):
#     p.resetJointState(robot_id , i,joint_angles[i])

# # _ = input("SEF _")
# # Step simulation to apply the joint angles
# for _ in range(500):  # Number of steps
#     p.stepSimulation()
#     time.sleep(1/240)  # Adjust sleep time as needed

# print(f"End-Effector Target Position: {iniial_pos}")
# print(f"End-Effector Actual Position: {p.getLinkState(robot_id, end_effector_link_index)[4]}")


# print("Initial positions set done !!")
# print("Now started moving !!")



def move_to_position_with_feedback(target_position, target_orientation):

    current_state = p.getLinkState(robot_id, end_effector_link_index)
    current_position = current_state[4]  # Current end-effector position
    current_orientation = current_state[5]  # Current end-effector orientation

    print(f"Current Position: {current_position}")
    print(f"Current Orientation: {current_orientation}")
            
    ik_joint_positions = p.calculateInverseKinematics(
        robot_id, 
        end_effector_link_index, 
        target_position, 
        target_orientation)
    
    for i in range(len(ik_joint_positions)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=ik_joint_positions[i] , 
        )

#     p.setJointMotorControlArray(
#     bodyUniqueId=robot_id, 
#     jointIndices=list(range(len(ik_joint_positions))),
#     controlMode=p.POSITION_CONTROL,
#     targetPositions=ik_joint_positions
# # )


    # for i in range(len(ik_joint_positions)):
    #     p.resetJointState(robot_id , i,ik_joint_positions[i])


    for _ in range(500): 
        p.stepSimulation()
        # time.sleep(1/240) 
    
    actual_position = p.getLinkState(robot_id, end_effector_link_index)[4]
    pos_diff = np.array(target_position) - np.array(actual_position)
    
    position_differences.append(pos_diff)

    # if np.linalg.norm(pos_diff) < tolerance:
    #     print(f"Target position reached within tolerance: {pos_diff}")
    # else:
    #     print(f"Target position not reached. Difference: {pos_diff}")

    return pos_diff


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
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=100,          
        aspect=1.0,     
        nearVal=0.1,     
        farVal=3.1       
    )

    # Capture RGB, depth, and segmentation mask
    width, height, img, _, _ = p.getCameraImage(
        width=800,           
        height=600,           
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL 
    )

    rgb_array = np.array(img)
    rgb_array = rgb_array[:, :, :3]  # No alpha chan

    bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)

    cv2.imwrite(file_name, bgr_array)

additional_image_dir = "rgb_images"
os.makedirs(additional_image_dir, exist_ok=True)


# # set - 1
camera_position = [0.06126845421106433, 0.4951337668615451, 0.6651930769671347]
camera_orientation = p.getQuaternionFromEuler([-1.5755163454721475, 0.009724033533159426, -2.1473608246676887])


# # set - 3
# camera_position = [0.1523211 , -0.75585572 , 0.46839562]
# camera_orientation = p.getQuaternionFromEuler([-1.83963878, -0.0405951  ,-1.06510052])

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
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 2]) # last param is just the line length 
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

draw_camera_direction(camera_position, camera_orientation)

for idx, pos in enumerate(positions):

    target_position = pos[:3]
    target_orientation = p.getQuaternionFromEuler(pos[3:])
    print(f"Moving to: {target_position}, orientation: {target_orientation}")

    move_to_position_with_feedback(target_position, target_orientation)
    image_name = os.path.join(additional_image_dir, f"camera_position_{idx}.png")
    capture_and_filter_arm(camera_position, camera_orientation, image_name)


position_differences = np.array(position_differences)
plt.figure(figsize=(10, 6))
plt.plot(position_differences)
plt.title("Difference between Expected and Actual Positions")
plt.xlabel("Position Index")
plt.ylabel("Difference (m)")
plt.legend(["X", "Y", "Z"])
plt.grid(True)
plt.show()
plt.savefig("../data/2.png")

p.disconnect()


