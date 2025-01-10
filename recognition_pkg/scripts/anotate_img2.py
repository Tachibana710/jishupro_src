import cv2
import json
import os
import base64
import numpy as np
import glob

is_drawing = False
binary_mask = None
brush_size = 5
mode = 'draw'  # Modes: 'draw' or 'erase'

def load_image(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    return image

def save_mask(mask_path, mask):
    cv2.imwrite(mask_path, mask * 255)  # Convert binary mask (0 or 1) to 8-bit grayscale for saving

def click_event(event, x, y, flags, param):
    global is_drawing, binary_mask, mode

    if event == cv2.EVENT_LBUTTONDOWN:
        is_drawing = True

    elif event == cv2.EVENT_MOUSEMOVE and is_drawing:
        if mode == 'draw':
            cv2.circle(binary_mask, (x, y), brush_size, 1, -1)
        elif mode == 'erase':
            cv2.circle(binary_mask, (x, y), brush_size, 0, -1)

        # Create a semi-transparent overlay
        overlay = param['image'].copy()
        overlay[binary_mask == 1] = (0, 255, 0)
        blended = cv2.addWeighted(param['image'], 0.7, overlay, 0.3, 0)
        cv2.imshow('Image Viewer', blended)

    elif event == cv2.EVENT_LBUTTONUP:
        is_drawing = False

def main():
    global binary_mask, mode

    image_dir = 'dataset/images'
    mask_dir = 'dataset/masks'

    if not os.path.exists(mask_dir):
        os.makedirs(mask_dir)

    image_paths = glob.glob(os.path.join(image_dir, '*.jpg'))

    for image_path in image_paths:
        image_name = os.path.basename(image_path)
        mask_path = os.path.join(mask_dir, image_name.replace('.jpg', '.png'))

        image = load_image(image_path)

        if os.path.exists(mask_path):
            binary_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE) // 255
            print(f"Loaded existing mask for {image_name}.")
        else:
            binary_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
            print(f"No mask found for {image_name}. Creating a new one.")

        cv2.imshow('Image Viewer', image)
        cv2.setMouseCallback('Image Viewer', click_event, {'image': image})

        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('s'):
                save_mask(mask_path, binary_mask)
                print(f"Saved mask for {image_name} at {mask_path}.")
                break
            elif key == ord('e'):
                mode = 'erase'
                print("Mode changed to erase.")
            elif key == ord('d'):
                mode = 'draw'
                print("Mode changed to draw.")

        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
