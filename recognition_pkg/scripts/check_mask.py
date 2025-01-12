import cv2
import numpy as np
import os
import glob

def load_mask(mask_path):
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"Mask not found: {mask_path}")
    return mask

def visualize_mask(mask, image=None):
    color_map = {
        0: (0, 0, 0),        # Background: Black
        1: (255, 255, 0),    # Class 1: Yellow
        2: (0, 0, 255),      # Class 2: Red
        3: (0, 255, 0),      # Class 3: Green
        4: (255, 0, 0)       # Class 4: Blue
    }

    height, width = mask.shape
    color_mask = np.zeros((height, width, 3), dtype=np.uint8)

    for value, color in color_map.items():
        color_mask[mask == value] = color

    if image is not None:
        blended = cv2.addWeighted(image, 0.7, color_mask, 0.3, 0)
        cv2.imshow('Mask Visualization', blended)
    else:
        cv2.imshow('Mask Visualization', color_mask)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    mask_dir = 'dataset/masks'
    image_dir = 'dataset/images'  # Optional: Load corresponding images for better visualization

    mask_paths = glob.glob(os.path.join(mask_dir, '*.png'))

    if not mask_paths:
        print("No masks found in the specified directory.")
        return

    for mask_path in mask_paths:
        print(f"Displaying mask: {mask_path}")
        mask = load_mask(mask_path)

        # Optional visualization
        image_name = os.path.basename(mask_path).replace('.png', '.jpg')
        image_path = os.path.join(image_dir, image_name)

        if os.path.exists(image_path):
            image = cv2.imread(image_path)
            visualize_mask(mask, image)
        else:
            visualize_mask(mask)

if __name__ == '__main__':
    main()
