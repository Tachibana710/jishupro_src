import os
import json
import base64
import cv2
import numpy as np
import glob

def json_to_jpg(json_dir, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    json_files = glob.glob(os.path.join(json_dir, '*.json'))

    for json_file in json_files:
        with open(json_file, 'r') as f:
            data = json.load(f)

        # Decode the image data
        image_data = base64.b64decode(data['data'])
        image = np.frombuffer(image_data, dtype=np.uint8)

        height = data['height']
        width = data['width']
        channels = 3
        image = np.array(image, dtype=np.uint8).reshape((height, width, channels))

        # Convert RGB to BGR (OpenCV default format)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Save as JPG
        output_path = os.path.join(output_dir, os.path.basename(json_file).replace('.json', '.jpg'))
        cv2.imwrite(output_path, image)
        print(f"Saved: {output_path}")

def main():
    json_dir = 'dataset/json'  # Directory containing JSON files
    output_dir = 'dataset/images'  # Directory to save the converted images

    json_to_jpg(json_dir, output_dir)

if __name__ == '__main__':
    main()
