import bagpy
from bagpy import bagreader
import pandas as pd

# Provide the path to your ROS bag file
bag = bagreader('data.bag')

# Extract Camera Info
# camera_info_csv = bag.message_by_topic('/camera/color/camera_info')
# camera_info_df = pd.read_csv(camera_info_csv)
# camera_info_df.to_csv('camera_info_2.csv', index=False)
# print(f"Extracted Camera Info: {camera_info_df.head()}")

# Extract Camera Image
# image_raw_csv = bag.message_by_topic('/camera/color/image_raw')
# image_raw_df = pd.read_csv(image_raw_csv)
# image_raw_df.to_csv('camera_image_2.csv', index=False)
# print(f"Extracted Camera Image: {image_raw_df.head()}")

# # Extract Joint States
# joint_states_csv = bag.message_by_topic('/xarm/joint_states')
# joint_states_df = pd.read_csv(joint_states_csv)
# joint_states_df.to_csv('joint_states_2.csv', index=False)
# print(f"Extracted Joint States: {joint_states_df.head()}")

# import bagpy
# from bagpy import bagreader
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# import os

# # Initialize the CvBridge for converting ROS image messages to OpenCV format
# bridge = CvBridge()

# # Provide the path to your ROS bag file
# bag = bagreader('2024-10-12-11-04-41.bag')

# image_save_dir = 'camera_images'
# os.makedirs(image_save_dir, exist_ok=True)

# # Iterate through messages in the `/camera/color/image_raw` topic and save them as images
# image_msgs = bag.message_by_topic('/camera/color/image_raw')
# for idx, msg in enumerate(bagpy.bagreader(image_msgs)):
#     # Convert ROS Image message to OpenCV image
#     img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
#     # Save the image as PNG or JPG format
#     image_filename = os.path.join(image_save_dir, f'image_{idx:04d}.png')
#     cv2.imwrite(image_filename, img)
#     print(f"Saved image: {image_filename}")


import bagpy
from bagpy import bagreader

# Provide the path to your ROS bag file
bag = bagreader('data.bag')

# Print all available topics in the bag
print(f"Available topics: {bag.topics}")

# Print detailed info of each topic, including message type, message count, and frequency
info = bag.topic_table
print(info)


# import bagpy
# from bagpy import bagreader
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# import os
# import pandas as pd

# # Initialize the CvBridge for converting ROS image messages to OpenCV format
# bridge = CvBridge()

# # Provide the path to your ROS bag file
# bag = bagreader('2024-10-12-11-04-41.bag')

# # Create a directory to save the images
# image_save_dir = 'camera_images'
# os.makedirs(image_save_dir, exist_ok=True)

# # Get the path to the CSV file that contains the image data
# image_raw_csv = bag.message_by_topic('/camera/color/image_raw')

# # Read the CSV data
# image_raw_df = pd.read_csv(image_raw_csv)

# # Iterate through the DataFrame and convert each image to OpenCV format and save
# for idx, row in image_raw_df.iterrows():
#     # Assuming 'data' column in the CSV contains the image data as a string of bytes
#     img_data = np.fromstring(row['data'], np.uint8)
    
#     # Decode the image data (replace 'data' with the correct column containing image data)
#     img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
    
#     # Save the image as PNG or JPG format
#     image_filename = os.path.join(image_save_dir, f'image_{idx:04d}.png')
#     cv2.imwrite(image_filename, img)
#     print(f"Saved image: {image_filename}")

# import rosbag
# from sensor_msgs.msg import Image, JointState
# from bisect import bisect_left

# # Open your ROS bag file
# # bag = bagreader('2024-10-12-11-04-41.bag')
# bag = rosbag.Bag('2024-10-12-11-04-41.bag')
# image_timestamps = []
# joint_state_timestamps = []

# # Extract image timestamps
# for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw']):
#     image_timestamps.append((t.to_sec(), msg))

# # Extract joint state timestamps
# for topic, msg, t in bag.read_messages(topics=['/xarm/joint_states']):
#     joint_state_timestamps.append((t.to_sec(), msg))

# print(len(image_timestamps))
# print(len(joint_state_timestamps))


# import csv

# # Create a CSV file to store the timestamps
# with open('timestamps.csv', mode='w', newline='') as file:
#     writer = csv.writer(file)
    
#     # Write header
#     writer.writerow(['Image Timestamp', 'Joint State Timestamp'])
    
#     # Get the maximum length to account for different numbers of image and joint state timestamps
#     max_length = max(len(image_timestamps), len(joint_state_timestamps))
    
#     # Write data to the CSV file
#     for i in range(max_length):
#         # Get the image timestamp if it exists, otherwise use an empty string
#         img_time = image_timestamps[i][0] if i < len(image_timestamps) else ''
        
#         # Get the joint state timestamp if it exists, otherwise use an empty string
#         joint_time = joint_state_timestamps[i][0] if i < len(joint_state_timestamps) else ''
        
#         writer.writerow([img_time, joint_time])

# print("Timestamps saved to 'timestamps.csv'.")