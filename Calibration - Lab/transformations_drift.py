import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) to a rotation matrix."""
    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

def combine_transformations(translation_a_to_b, rpy_a_to_b, translation_b_to_c, rpy_b_to_c):
    """
    Combine transformations from A to B and B to C to get transformation from A to C.
    """
    R_a_to_b = euler_to_rotation_matrix(*rpy_a_to_b)
    R_b_to_c = euler_to_rotation_matrix(*rpy_b_to_c)

    R_a_to_c = R_a_to_b @ R_b_to_c  
    translation_a_to_c = translation_a_to_b + R_a_to_b @ translation_b_to_c

    rpy_a_to_c = R.from_matrix(R_a_to_c).as_euler('xyz')
    return translation_a_to_c, rpy_a_to_c


def invert_transformation(translation, rpy):
    """
    Compute the inverse of a transformation.
    """
    R_mat = euler_to_rotation_matrix(*rpy)
    R_mat_inv = R_mat.T
    
    translation_inv = -R_mat_inv @ np.array(translation)
    
    rpy_inv = R.from_matrix(R_mat_inv).as_euler('xyz')
    
    return translation_inv, rpy_inv

def quat_to_euler(quat):
    rotation = R.from_quat(quat)
    euler = rotation.as_euler('xyz', degrees=False)
    return euler


# # Transformation from base to link_base
# translation_base_to_link_base = [0.021131, 0.0016302, -0.056488]
# rpy_base_to_link_base = [0, 0, 0]  # Identity rotation

# # Compute inverse transformation
# translation_link_base_to_base, rpy_link_base_to_base = invert_transformation(
#     translation_base_to_link_base, rpy_base_to_link_base
# )


# print("\nInverse Translation from base to link_base:", translation_link_base_to_base)
# print("Inverse RPY from base to link_base:", rpy_link_base_to_base)


# -0.021131, -0.0016302, 0.056488

"""
args="0.701274 1.0029 -0.00991212   -0.0478799 -0.0673292 -0.789239 0.608503 link_base camera_link" />
"""


# quat = (-0.0478799 ,-0.0673292, -0.789239 ,0.608503  )

# euler_angles = quat_to_euler(quat)
# print("Euler Angles (rad):", euler_angles)


translation_a_to_b = [0.701274, 1.0029, -0.00991212]  
rpy_a_to_b =[0.04863361 ,-0.1581762,  -1.83183581]  

# from camera_link to camera_optical_colour_frame -> found using the rosrun query 
translation_b_to_c = [0 , -0.059 , 0]  
rpy_b_to_c = [-1.567 , -0.002 , -1.570]  # pi/2 rot clear here !!

translation_a_to_c, rpy_a_to_c = combine_transformations(
    translation_a_to_b, rpy_a_to_b, translation_b_to_c, rpy_b_to_c)

print(translation_a_to_c ,rpy_a_to_c )

"""
optical_frame_T_base = optical_frame_T_camera_link * camera_link_T_link_base * link_base_T_base
"""

"""
base = pybullet
A = link base (arm)
B = camera link 
C = optical frame 
"""

# Transformation from base to link_base
# translation_base_to_link_base = [0.021131, 0.0016302, -0.056488]
# rpy_base_to_link_base = [0, 0, 0]  # Identity rotation


# # Transformation from link_base to camera_link
# translation_link_base_to_camera_link = [0.701274, 1.0029, -0.00991212]
# rpy_link_base_to_camera_link = [2.97629, -3.13523, 1.31308]

# # Transformation from camera_link to camera_optical_colour_frame
# translation_camera_link_to_optical_frame = [0, -0.059, 0]
# rpy_camera_link_to_optical_frame = [-1.567, -0.002, -1.570] 


"""
combine_transformations() : 
"""
# # Step 1: Combine link_base -> camera_link -> camera_optical_colour_frame
# translation_link_base_to_optical_frame, rpy_link_base_to_optical_frame = combine_transformations(

#     translation_link_base_to_camera_link,
#     rpy_link_base_to_camera_link,
#     translation_camera_link_to_optical_frame,
#     rpy_camera_link_to_optical_frame
# )


# # print(translation_link_base_to_optical_frame)
# # print(rpy_camera_link_to_optical_frame)


# # Step 2: Combine base -> link_base -> optical_frame
# translation_base_to_optical_frame, rpy_base_to_optical_frame = combine_transformations(

#     translation_base_to_link_base,
#     rpy_base_to_link_base,
#     translation_link_base_to_optical_frame,
#     rpy_link_base_to_optical_frame)




# # Final results
# print("\n\nTranslation from base to camera_optical_colour_frame:", translation_base_to_optical_frame)
# print("RPY from base to camera_optical_colour_frame (in radians):", rpy_base_to_optical_frame)


# # Step 1: Combine link_base -> camera_link -> camera_optical_colour_frame
# translation_link_base_to_optical_frame, rpy_link_base_to_optical_frame = combine_transformations(
#     [0.701274, 1.0029, -0.00991212],  # link_base to camera_link translation
#     [2.97629, -3.13523, 1.31308],     # link_base to camera_link RPY
#     [0, -0.059, 0],                   # camera_link to optical_frame translation
#     [-1.567, -0.002, -1.570]          # camera_link to optical_frame RPY
# )

# # Step 2: Combine base -> link_base -> camera_optical_colour_frame
# translation_base_to_optical_frame, rpy_base_to_optical_frame = combine_transformations(
#     [0.021131, 0.0016302, -0.056488],  # base to link_base translation
#     [0, 0, 0],                         # base to link_base RPY
#     translation_link_base_to_optical_frame,
#     rpy_link_base_to_optical_frame
# )

# # Final results
# print("\nTranslation from base to camera_optical_colour_frame:", translation_base_to_optical_frame)
# print("RPY from base to camera_optical_colour_frame (in radians):", rpy_base_to_optical_frame)