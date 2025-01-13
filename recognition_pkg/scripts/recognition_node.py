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
from tf2_msgs.msg import TFMessage
import geometry_msgs
import pyrealsense2 as rs
from realsense2_camera_msgs.msg import Extrinsics
from sensor_msgs.msg import CameraInfo
import cv2
import std_msgs

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

        self.color_intrin = None
        def color_camera_info_callback(msg):
            self.color_intrin = rs.intrinsics()
            self.color_intrin.width = msg.width
            self.color_intrin.height = msg.height
            self.color_intrin.ppx = msg.k[2]
            self.color_intrin.ppy = msg.k[5]
            self.color_intrin.fx = msg.k[0]
            self.color_intrin.fy = msg.k[4]
            self.color_intrin.coeffs = [0, 0, 0, 0, 0]
        self.color_camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera/color/camera_info',
            color_camera_info_callback,
            10
        )

        self.extrin = rs.extrinsics()

        self.extrin.rotation = [
            0.9999729990959167, -0.005194165278226137, -0.005202271044254303,
            0.005191181320697069, 0.9999863505363464, -0.0005869486485607922,
            0.005205248948186636, 0.0005599268479272723, 0.9999862909317017
        ]
        self.extrin.translation = [0.015142371878027916, -6.709677109029144e-05, 1.8719321815297008e-05]

        def extrin_callback(msg):
            self.extrin = msg

        self.extrin_sub = self.create_subscription(
            Extrinsics,
            'camera/camera/extrinsics/depth_to_color',
            extrin_callback,
            10
        )

        # self.observ_y_pub = self.create_publisher(
        #     std_msgs.msg.Float32,
        #     'recognition/observation/y',
        #     10
        # )

        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # self.timer = self.create_timer(1.0, self.get_extrin)


        self.depth_img = None
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # self.timer = self.create_timer(1, self.publish_tf)

        # モデルの準備
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.load_model()
        self.image_transform = self.get_image_transform()

    # def get_extrin(self):
    #     try:
    #         # transform = self.tf_buffer.lookup_transform("camera_depth_optical_frame", "camera_color_optical_frame", rclpy.time.Time())
    #         # self.extrin = rs.rs2_extrinsics()
    #         # self.extrin.rotation = [
    #         #     transform.transform.rotation.x,
    #         #     transform.transform.rotation.y,
    #         #     transform.transform.rotation.z,
    #         #     transform.transform.rotation.w
    #         # ]
    #         # self.extrin.translation = [
    #         #     transform.transform.translation.x,
    #         #     transform.transform.translation.y,
    #         #     transform.transform.translation.z
    #         # ]
    #         depth_to_camera = self.tf_buffer.lookup_transform("camera_depth_optical_frame", "camera_link", rclpy.time.Time())
    #         color_to_camera = self.tf_buffer.lookup_transform("camera_color_optical_frame", "camera_link", rclpy.time.Time())
    #         color_to_depth = 

    #     except Exception as e:
    #         self.get_logger().error(f"Failed to get extrinsics: {e}")

    def load_model(self):
        from torchvision import models
        model = models.segmentation.deeplabv3_resnet50(pretrained=False, num_classes=5)
        model_data_path = os.path.join("/home/taka/mech/jishu_pro/jishupro_ws/src/recognition_pkg/scripts", "segmentation_model.pth")
        model.load_state_dict(torch.load(model_data_path, map_location=self.device))
        model = model.to(self.device)
        model.eval()
        return model

    def depth_callback(self, msg):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_img = cv_depth
        # self.publish_tf()
        # self.get_logger().info('depth image received')

    def publish_tf(self):
        
        if self.depth_img is None or self.depth_scale is None or self.depth_intrin is None or self.color_intrin is None or self.extrin is None:
            if self.depth_img is None:
                self.get_logger().info('depth image is None')
            if self.depth_scale is None:
                self.get_logger().info('depth scale is None')
            if self.depth_intrin is None:
                self.get_logger().info('depth intrin is None')
            if self.color_intrin is None:
                self.get_logger().info('color intrin is None')
            if self.extrin is None:
                self.get_logger().info('extrin is None')
            return
        colormap = [
            [0,0,0],
            [255,255,0],
            [0,0,255],
            [0,255,0],
            [255,0,0],
        ]
        # resized_mask = cv2.resize(self.mask, (self.depth_img.shape[1], self.depth_img.shape[0]), interpolation=cv2.INTER_NEAREST)
        def publish_object_tf(i):
            def pixel_to_3d(pixel_x,pixel_y):
                color_point = rs.rs2_deproject_pixel_to_point(self.color_intrin, [pixel_x, pixel_y], 1)
                depth_point = rs.rs2_transform_point_to_point(self.extrin, color_point)
                # depth_point = color_point
                depth_pixel = rs.rs2_project_point_to_pixel(self.depth_intrin, depth_point)
                depth_pixel = [int(depth_pixel[0]), int(depth_pixel[1])]
                depth = self.depth_img[depth_pixel[1]][depth_pixel[0]] * self.depth_scale
                # print(self.depth_intrin)
                # depth = self.depth_img[pixel_y][pixel_x] * self.depth_scale
                result = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [depth_pixel[0], depth_pixel[1]], depth)
                return result

            pointcloud = []
            # for x in range(0, self.mask.shape[1], 10):
            #     for y in range(0, self.mask.shape[0], 10):
            #        if np.array_equal(self.mask[y, x], colormap[i]):
            #             p = pixel_to_3d(x, y)
            #             if p is not None:
            #                 pointcloud.append(p)

            mask_indices = np.where(np.all(self.mask == colormap[i], axis=-1))
            if len(mask_indices[0]) == 0:
                return
            # mean_x = np.mean(mask_indices[1])
            # mean_y = np.mean(mask_indices[0])
            # center = pixel_to_3d(int(mean_x), int(mean_y))
            for y, x in zip(mask_indices[0], mask_indices[1]):
                p = pixel_to_3d(x, y)
                if p is not None:
                    pointcloud.append(p)

            # # if len(pointcloud) > 0:
            # #     center = np.mean(pointcloud, axis=0)
            # # else:
            # #     return
            if len(pointcloud) == 0:
                return


            center = np.mean(pointcloud, axis=0)

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_depth_optical_frame"
            t.child_frame_id = "object"+str(i)
            t.transform.translation.x = center[0]
            t.transform.translation.y = center[1]
            t.transform.translation.z = center[2]
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)

        for i in range(1, 5):
            publish_object_tf(i)
        # print('published tf')
        # publish_object_tf(1)




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
            [0,0,0],
            [255,255,0],
            [0,0,255],
            [0,255,0],
            [255,0,0],
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
