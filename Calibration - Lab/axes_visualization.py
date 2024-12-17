import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/xarm7_robot.urdf", [0, 0, 0], useFixedBase=True)


def draw_axes(robot_id, link_index, length = 0.4):

    link_state = p.getLinkState(robot_id, link_index)
    position = link_state[0] 
    orientation = link_state[1]  

    axis_colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  
    
    print(f"Origin of link {link_index}: {position}")


    for i in range(3):

        if i == 0:  # X axis - Red
            axis_end = [position[0] + length, position[1], position[2]]
        elif i == 1:  # Y axis - Green
            axis_end = [position[0], position[1] + length, position[2]]
        else:  # Z axis - Blue
            axis_end = [position[0], position[1], position[2] + length]

        # Draw the axis lines
        p.addUserDebugLine(position, axis_end, lineColorRGB=axis_colors[i], lineWidth=5, lifeTime=0) 


joint_angles = np.array([0.5181461, -0.98889416, 0.76654553, 0.50665468, 2.2308166, -0.59086448, -2.04378772])

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

move_to_joint_angles(joint_angles)

num_links = p.getNumJoints(robot_id)

for link_index in range(0, 1):  
    draw_axes(robot_id, link_index)

while True:
    p.stepSimulation()
    time.sleep(1 / 240)



