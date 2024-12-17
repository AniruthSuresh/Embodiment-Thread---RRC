# import pandas as pd
# import numpy as np

# # Load the CSV file into a DataFrame
# csv_file_path = 'cleaned_js.csv'  # Update with your actual file path
# data = pd.read_csv(csv_file_path)

# # Extract the 'joint_states' column and convert it to a NumPy array
# # Split the string by comma and convert each element to a float
# joint_states_array = np.array(data['joint_states'].str.split(', ').tolist(), dtype=float)

# # Check the shape of the resulting array
# print(f"Shape of joint_states array: {joint_states_array.shape}")

# # Save the joint_states_array if needed
# np.save('joint_states.npy', joint_states_array)  # Save as .npy file

# # Optionally, print the first few rows of the array
# print(joint_states_array[:5])  # Print the first 5 rows
# import numpy as np
# from scipy.spatial.transform import Rotation as R

# def euler_to_quaternion(roll, pitch, yaw):
#     # Create a rotation object from Euler angles (XYZ convention)
#     rotation = R.from_euler('xyz', [roll, pitch, yaw])
    
#     # Convert the rotation object to a quaternion
#     quaternion = rotation.as_quat()  # returns [x, y, z, w]
    
#     return quaternion

# # Given Euler angles in radians
# camera_orientation = [2.97629, -3.13523, 1.31308]  # [roll, pitch, yaw]

# # Convert to quaternion
# quaternion = euler_to_quaternion(*camera_orientation)

# # Print the resulting quaternion
# print("Quaternion (x, y, z, w):", quaternion)


