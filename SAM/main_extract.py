import os
import cv2
import numpy as np
import torch
import re
from segment_anything import SamPredictor, sam_model_registry
from tqdm import tqdm

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")


sam_checkpoint = "./segment-anything/sam_vit_h_4b8939.pth"
model_type = "vit_h"  # SAM model type
sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
sam.to(device)
predictor = SamPredictor(sam)

rgb_folder = "../../Complete_Data/xArm/RGB_Now/"
mask_folder = "../../Complete_Data/xArm/Mask_now/"
output_folder = "../../Complete_Data/xArm/Mask_SAM/"

os.makedirs(output_folder, exist_ok=True)

num_space = 450  # Spacing for selecting points
starting_number = 696  # User-defined starting number
counter = starting_number  # Initialize counter with the starting number


def extract_number(filename):
    """
    Extracts the first number found in the filename.
    """
    numbers = re.findall(r'\d+', filename)
    return int(numbers[0]) if numbers else float('inf')

# Sort files in both folders by the extracted number
rgb_files = sorted(
    [f for f in os.listdir(rgb_folder) if f.endswith(".png")],
    key=extract_number
)
mask_files = sorted(
    [f for f in os.listdir(mask_folder) if f.endswith(".png")],
    key=extract_number
)

# Ensure the same number of files in both folders
if len(rgb_files) != len(mask_files):
    print("Error: The number of files in RGB and Mask folders do not match.")
    exit()

for rgb_file, mask_file in tqdm(zip(rgb_files, mask_files), desc="Processing files", unit="file", total=len(rgb_files)):
    rgb_image_path = os.path.join(rgb_folder, rgb_file)
    mask_image_path = os.path.join(mask_folder, mask_file)

    rgb_image = cv2.imread(rgb_image_path)
    mask_image = cv2.imread(mask_image_path, cv2.IMREAD_GRAYSCALE)

    if rgb_image is None or mask_image is None:
        print(f"Error loading {rgb_file} or {mask_file}. Skipping...")
        continue

    _, binary_mask = cv2.threshold(mask_image, 50, 255, cv2.THRESH_BINARY)

    skeleton = cv2.ximgproc.thinning(binary_mask)
    bold_skeleton = cv2.dilate(skeleton, np.ones((3, 3), np.uint8), iterations=3)

    skeleton_points = np.column_stack(np.where(bold_skeleton > 0))

    input_points = np.array([[point[1], point[0]] for point in skeleton_points])  # (x, y) format
    input_labels = np.ones(input_points.shape[0])  # Foreground labels

    for point in input_points[::num_space]:
        cv2.circle(rgb_image, (point[0], point[1]), radius=3, color=(0, 0, 255), thickness=1)

    predictor.set_image(rgb_image)
    torch.cuda.empty_cache()  # Clear unused memory
    masks, _, _ = predictor.predict(
        point_coords=input_points[::num_space],
        point_labels=input_labels[::num_space],
        multimask_output=False,  
    )

    segmented_mask = masks[0]
    segmented_mask_uint8 = (segmented_mask * 255).astype(np.uint8) if segmented_mask.max() == 1 else segmented_mask.astype(np.uint8)

    kernel = np.ones((5, 5), np.uint8)  
    closed_mask = cv2.morphologyEx(segmented_mask_uint8, cv2.MORPH_CLOSE, kernel)

    output_path = os.path.join(output_folder, f"{counter:04d}.png")  # Name with padded numbers
    cv2.imwrite(output_path, closed_mask)
    print(f"Processed and saved mask for {rgb_file} and {mask_file} as {counter:04d}.png.")

    counter += 1  # Increment the counter for the next file

print("Processing complete. All predicted masks are saved.")
