#!/usr/bin/env python3


import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
import torch
import os
import tf2_ros
import geometry_msgs
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo

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
        self.mask = None

        self.bridge = CvBridge()

        # pipeline = rs.pipeline()
        # config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # profile = pipeline.start(config)
        # self.depth_sensor = profile.get_device().first_depth_sensor()
        # self.depth_scale = self.depth_sensor.get_depth_scale()
        # self.depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        # pipeline.stop()

        self.depth_scale = None
        self.depth_intrin = None
        def camera_info_callback(msg):
            self.depth_scale = 0.001
            self.depth_intrin = rs.intrinsics()
            self.depth_intrin.width = msg.width
            self.depth_intrin.height = msg.height
            self.depth_intrin.ppx = msg.k[2]
            self.depth_intrin.ppy = msg.k[5]
            self.depth_intrin.fx = msg.k[0]
            self.depth_intrin.fy = msg.k[4]
            self.depth_intrin.coeffs = [0, 0, 0, 0, 0]
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera/depth/camera_info',
            camera_info_callback,
            10
        )

        self.depth_img = None
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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

    def depth_callback(self, msg):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_img = cv_depth
        # self.get_logger().info('depth image received')

    def publish_tf(self):
        def pixel_to_3d(x,y):
            depth = self.depth_img[y][x] * self.depth_scale
            # print(self.depth_intrin)
            result = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [x, y], depth)
            return result

        pointcloud = []
        for x in range(0, 640, 10):
            for y in range(0, 480, 10):
                if np.array_equal(self.mask[y, x], [255, 255, 255]):
                    depth = self.depth_img[y, x]
                    if depth > 0:
                        pointcloud.append(pixel_to_3d(x, y))

        if len(pointcloud) > 0:
            center = np.mean(pointcloud, axis=0)
        else:
            return


        center = np.mean(pointcloud, axis=0)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = "object"
        t.transform.translation.x = center[0]
        t.transform.translation.y = center[1]
        t.transform.translation.z = center[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)




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
        self.mask = np.array(colored_mask)
        # カラーマスクをROS2のImageメッセージに変換
        cv_colored_mask = np.array(colored_mask)
        publish_msg = self.bridge.cv2_to_imgmsg(cv_colored_mask, encoding='rgb8')

        # トピックにPublish
        self.publisher_.publish(publish_msg)

        self.publish_tf()


def main(args=None):
    rclpy.init(args=args)
    segmentation_node = SegmentationNode()
    rclpy.spin(segmentation_node)
    segmentation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
