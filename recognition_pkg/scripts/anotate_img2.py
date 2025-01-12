import cv2
import os
import numpy as np
import glob

is_drawing = False
binary_mask = None
brush_size = 3
current_class = 1  # Initial class is 1 (can be changed dynamically)


def load_image(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    return image


def save_mask(mask_path, mask):
    cv2.imwrite(mask_path, mask)  # Save mask as an 8-bit single-channel image


def click_event(event, x, y, flags, param):
    global is_drawing, binary_mask, current_class

    if event == cv2.EVENT_LBUTTONDOWN:
        is_drawing = True
        cv2.circle(binary_mask, (x, y), brush_size, current_class, -1)

        # Create a semi-transparent overlay
        overlay = param['image'].copy()
        overlay[binary_mask == 1] = (0, 255, 255)  # Class 1: Yellow (BGR)
        overlay[binary_mask == 2] = (255, 0, 0)    # Class 2: Blue (BGR)
        overlay[binary_mask == 3] = (0, 255, 0)    # Class 3: Green (BGR)
        overlay[binary_mask == 4] = (0, 0, 255)    # Class 4: Red (BGR)

        blended = cv2.addWeighted(param['image'], 0.7, overlay, 0.3, 0)
        cv2.imshow('Image Viewer', blended)

    elif event == cv2.EVENT_MOUSEMOVE and is_drawing:
        cv2.circle(binary_mask, (x, y), brush_size, current_class, -1)

        # Create a semi-transparent overlay
        overlay = param['image'].copy()
        overlay[binary_mask == 1] = (0, 255, 255)  # Class 1: Yellow (BGR)
        overlay[binary_mask == 2] = (255, 0, 0)    # Class 2: Blue (BGR)
        overlay[binary_mask == 3] = (0, 255, 0)    # Class 3: Green (BGR)
        overlay[binary_mask == 4] = (0, 0, 255)    # Class 4: Red (BGR)

        blended = cv2.addWeighted(param['image'], 0.7, overlay, 0.3, 0)
        cv2.imshow('Image Viewer', blended)

    elif event == cv2.EVENT_LBUTTONUP:
        is_drawing = False


def main():
    global binary_mask, current_class

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
            binary_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
            # print(f"Loaded existing mask for {image_name}.")
            print(f"mask for {image_name} is already exist.")
            continue
        else:
            binary_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
            print(f"No mask found for {image_name}. Creating a new one.")

        cv2.imshow('Image Viewer', image)
        cv2.setMouseCallback('Image Viewer', click_event, {'image': image})

        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC to quit
                break
            elif key == ord('s'):  # Save mask
                save_mask(mask_path, binary_mask)
                print(f"Saved mask for {image_name} at {mask_path}.")
                current_class = 1  # Reset class
                break
            elif key in [ord('1'), ord('2'), ord('3'), ord('4')]:
                current_class = int(chr(key))  # Change class based on key press
                print(f"Current class changed to {current_class}.")

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
