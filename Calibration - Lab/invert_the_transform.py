import numpy as np
from scipy.spatial.transform import Rotation as R

def invert_the_transform(quaternion , translation):
    """
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
    """

    rotation = R.from_quat(quaternion) # by deafult , scalar last is assumed !
    R_matrix = rotation.as_matrix()

    R_inv = R_matrix.T 
    t_inv = -R_inv @ translation 

    rpy_inv = R.from_matrix(R_inv).as_euler('xyz', degrees=False)  

    print("Inverse Translation:", t_inv)
    print("Inverse Rotation (RPY in radians):", rpy_inv)


quaternion = [-0.0478799, -0.0673292, -0.789239, 0.608503]
translation = np.array([0.701274, 1.0029, -0.00991212])


# invert_the_transform(quaternion , translation)


def euler_to_rotation_matrix(roll, pitch, yaw):

    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

def combine_transformations(translation_a_to_b, rpy_a_to_b, translation_b_to_c, rpy_b_to_c):
    """
    Combine transformations from A to B and B to C to get transformation from A to C.
    A = link_base , B = camera_link , C = camera_optical_colour_frame
    """

    R_a_to_b = euler_to_rotation_matrix(*rpy_a_to_b)
    R_b_to_c = euler_to_rotation_matrix(*rpy_b_to_c)

    R_a_to_c = R_a_to_b @ R_b_to_c  

    translation_a_to_c = translation_a_to_b + R_a_to_b @ translation_b_to_c

    rpy_a_to_c = R.from_matrix(R_a_to_c).as_euler('xyz')

    return translation_a_to_c, rpy_a_to_c

# from pybullet_base to link_base
# rpy_py_base_to_link_base = np.eye(3)
# translation_py_base_to_link_base = [2.97629, -3.13523, 1.31308]  



# from link_base to camera_link 
translation_a_to_b = [0.701274, 1.0029, -0.00991212]  
rpy_a_to_b = [2.97629, -3.13523, 1.31308]  

# from camera_link to camera_optical_colour_frame -> found using the rosrun query 
translation_b_to_c = [0 , -0.059 , 0]  
rpy_b_to_c = [-1.567 , -0.002 , -1.570]  # pi/2 rot clear here !!

translation_a_to_c, rpy_a_to_c = combine_transformations(
    translation_a_to_b, rpy_a_to_b, translation_b_to_c, rpy_b_to_c)

print("\n\nTranslation from A to C:", translation_a_to_c)
print("RPY from A to C (in radians):", rpy_a_to_c)
print("\n\n")


