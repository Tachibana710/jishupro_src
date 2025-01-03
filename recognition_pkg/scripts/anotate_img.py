import cv2
import json
import os
import base64
import numpy as np
import glob

target_position = None

def load_image_from_json(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)

    image_data = base64.b64decode(data['data'])
    image = np.frombuffer(image_data, dtype=np.uint8)
    
    # OpenCVで画像に変換
    height = data['height']
    width = data['width']
    channels = 3 
    image = np.array(image, dtype=np.uint8).reshape((height, width, channels))

    # RGB→BGR
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    return image, data

def save_target_to_json(file_path, data, target):
    data['target_value'] = target
    with open(file_path, 'w') as f:
        json.dump(data, f, indent=4)

def click_event(event, x, y, flags, param):
    global target_position

    if event == cv2.EVENT_LBUTTONDOWN:
        target_position = (x, y)
        print(f"Clicked position: {target_position}")

        # 印を描画
        image_with_marker = param.copy()
        cv2.circle(image_with_marker, target_position, 5, (0, 0, 255), -1)
        cv2.imshow('Image Viewer', image_with_marker)

def main(data_file):
    global target_position

    if not os.path.exists(data_file):
        print(f"File not found: {data_file}")
        return

    # 画像データ読み込み
    image, data = load_image_from_json(data_file)

    if data.get('target_value'):
        print(f"Existing target value: {data['target_value']}")
        target_position = tuple(data['target_value'])
        cv2.circle(image, target_position, 5, (0, 0, 255), -1)
    else:
        print("No target value found. Click on the image to set it.")

    cv2.imshow('Image Viewer', image)
    cv2.setMouseCallback('Image Viewer', click_event, image)

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        elif key == ord('s') and target_position:
            # target_valueを書き込んで保存
            save_target_to_json(data_file, data, target_position)
            print(f"Saved target value: {target_position}")
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    data_files = glob.glob('dataset/*.json')

    for data_file in data_files:
        main(data_file)
