#!/usr/bin/env python3


import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
import torch
import os

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        # ROS2パラメータ
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Image,
            'recognition/segmentation/mask',
            10
        )
        self.bridge = CvBridge()

        # モデルの準備
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.load_model()
        self.image_transform = self.get_image_transform()

    def load_model(self):
        from torchvision import models
        model = models.segmentation.deeplabv3_resnet50(pretrained=False, num_classes=21)
        model_data_path = os.path.join("/home/taka/mech/jishu_pro/jishupro_ws/src/recognition_pkg/scripts", "segmentation_model.pth")
        model.load_state_dict(torch.load(model_data_path, map_location=self.device))
        model = model.to(self.device)
        model.eval()
        return model

    def get_image_transform(self):
        from torchvision import transforms
        return transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def image_callback(self, msg):
        # ROS2のImageメッセージをPIL形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        input_image = PILImage.fromarray(cv_image)

        # 前処理
        input_tensor = self.image_transform(input_image).unsqueeze(0).to(self.device)

        # 推論
        with torch.no_grad():
            output = self.model(input_tensor)["out"]
            predicted_mask = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()

        # カラーマップの適用
        colormap = [
            [0, 0, 0],
            [255, 255, 255],
        ]

        def apply_colormap(mask, colormap):
            color_mask = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
            for class_idx in range(len(colormap)):
                color_mask[mask == class_idx] = colormap[class_idx]
            return color_mask

        colored_mask = apply_colormap(predicted_mask, colormap)
        colored_mask = PILImage.fromarray(colored_mask)
        colored_mask = colored_mask.resize((msg.width, msg.height))  # 元の画像サイズにリサイズ

        # カラーマスクをROS2のImageメッセージに変換
        cv_colored_mask = np.array(colored_mask)
        publish_msg = self.bridge.cv2_to_imgmsg(cv_colored_mask, encoding='rgb8')

        # トピックにPublish
        self.publisher_.publish(publish_msg)


def main(args=None):
    rclpy.init(args=args)
    segmentation_node = SegmentationNode()
    rclpy.spin(segmentation_node)
    segmentation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
