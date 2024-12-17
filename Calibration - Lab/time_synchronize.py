import os
import rosbag
import pandas as pd
from sensor_msgs.msg import Image, JointState
from bisect import bisect_left
import cv2
import numpy as np

bag = rosbag.Bag('data.bag')

image_timestamps = []
joint_state_timestamps = []

for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw']):
    image_timestamps.append((t.to_sec(), msg))

for topic, msg, t in bag.read_messages(topics=['/xarm/joint_states']):
    joint_state_timestamps.append((t.to_sec(), msg))


# print(len(image_timestamps))
# print(len(joint_state_timestamps))

def find_closest_joint_state(image_time, joint_state_times):
    index = bisect_left([t[0] for t in joint_state_times], image_time)
    if index == 0:
        return joint_state_times[0]  # Closest to the start
    if index == len(joint_state_times):
        return joint_state_times[-1]  # Closest to the end
    before = joint_state_times[index - 1]
    after = joint_state_times[index]
    if abs(image_time - before[0]) < abs(image_time - after[0]):
        return before
    else:
        return after

output_dir = 'cleaned_image_2'
os.makedirs(output_dir, exist_ok=True)

joint_states_list = []

for i, (image_time, image_msg) in enumerate(image_timestamps):
    closest_joint_state = find_closest_joint_state(image_time, joint_state_timestamps)
    
    image_data = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)

    image_filename = f"{output_dir}/image_{i:04d}.png"  
    cv2.imwrite(image_filename, image_data)

    joint_states_list.append((image_time, closest_joint_state[1].position))

joint_states_df = pd.DataFrame(joint_states_list, columns=['timestamp', 'joint_states'])
joint_states_df['joint_states'] = joint_states_df['joint_states'].apply(lambda x: ', '.join(map(str, x))) 
joint_states_df.to_csv('cleaned_js_2.csv', index=False)

bag.close()

print(f"Saved {len(image_timestamps)} synchronized images to '{output_dir}' and joint states to 'cleaned_js_2.csv'.")

