import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2
import pyrealsense2 as rs


class PlaneDetectionNode(Node):
    def __init__(self):
        super().__init__('plane_detection_node')

        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

        # サブスクライバ設定
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/plane_marker', 10)
        self.point_cloud_pub = self.create_publisher(point_cloud2.PointCloud2, '/point_cloud', 10)

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

        self.last_update_time = self.get_clock().now()
        # # カメラの内部パラメータ (仮に設定。CameraInfoトピックを購読して取得してもよい)
        # self.fx = 525.0  # 焦点距離 x
        # self.fy = 525.0  # 焦点距離 y
        # self.cx = 319.5  # 画像中心 x
        # self.cy = 239.5  # 画像中心 y

    def publish_plane_marker(self, plane_model, inliers, point_cloud):
        # 平面のパラメータを取得 (ax + by + cz + d = 0)
        a, b, c, d = plane_model

        # 平面上の点を計算 (中心点を使う)
        inlier_points = point_cloud[inliers]
        center = np.mean(inlier_points, axis=0)

        # 法線ベクトル
        normal = np.array([a, b, c])

        # 平面の範囲を設定 (例: 1m x 1m の平面)
        plane_size = 1.0
        u = np.cross(normal, [0, 0, 1])
        if np.linalg.norm(u) < 1e-6:  # 法線がz軸と平行な場合
            u = np.array([1, 0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        v /= np.linalg.norm(v)

        # 四隅の頂点を計算
        corners = [
            center + plane_size * (u + v),
            center + plane_size * (u - v),
            center - plane_size * (u + v),
            center - plane_size * (u - v)
        ]

        # Markerメッセージを作成
        marker = Marker()
        marker.header.frame_id = 'camera_depth_optical_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'plane'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.5  # 半透明
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # 平面を三角形で構成
        marker.points = []
        for i in [0, 1, 2, 0, 2, 3]:
            p = corners[i]
            point = Point()
            point.x, point.y, point.z = p
            marker.points.append(point)

        # パブリッシュ
        self.marker_pub.publish(marker)


    def depth_callback(self, msg):
        # 1秒に1回のみ処理
        if (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9 < 1.0:
            return
        # Depth画像を変換
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_image = depth_image.astype(np.float32) / 1000.0  # mmからmへ変換

        # 3Dポイントクラウドに変換
        point_cloud = self.depth_to_pointcloud(depth_image)

        # PointCloudメッセージを作成
        print(msg.header.frame_id)
        point_cloud_msg = point_cloud2.create_cloud_xyz32(msg.header, point_cloud)
        self.point_cloud_pub.publish(point_cloud_msg)

        # 平面検出 (RANSAC)
        plane_model, inliers = self.detect_plane(point_cloud)

        if plane_model is not None:
            # # 平面の法線ベクトルを取得
            # normal = plane_model[:3]

            # # カメラの回転（ピッチとロール）を計算
            # if plane_model[3] < 0:
            #     normal = -normal
            # rotation = self.calculate_camera_orientation(normal)

            # # 平面モデルと原点の距離を計算
            # distance_to_origin = abs(plane_model[3]) / np.linalg.norm(plane_model[:3])

            rot, trans = self.calculate_camera_transform(plane_model, point_cloud[inliers])

            # TFをブロードキャスト
            self.broadcast_tf(rot,trans)

    def calculate_camera_transform(self, plane_model, offset_points):
        '''
            plane_model : z = 0 の平面
            offset_points : y = 0 上の二点
        '''

        normal = plane_model[:3]
        if plane_model[3] < 0:
            normal = -normal

        # plane_modelと合致させる
        z_axis = np.array([0, 0, 1])
        rotation_vector = np.cross(normal,z_axis)
        rotation_vector /= np.linalg.norm(rotation_vector)
        angle = np.arccos(np.dot(z_axis, normal))
        K = np.array([
            [0, -rotation_vector[2], rotation_vector[1]],
            [rotation_vector[2], 0, -rotation_vector[0]],
            [-rotation_vector[1], rotation_vector[0], 0]
        ])
        rotation = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

        z = abs(plane_model[3]) / np.linalg.norm(normal)

        # offset_pointsを合致させる

        yaw = 1.57
        rotation_aroud_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        rotation = rotation_aroud_z @ rotation

        translation = [0, 0, z]

        return rotation, translation
        

    def depth_to_pointcloud(self, depth_image):
        h, w = depth_image.shape
        points = []
        for v in range(h):
            for u in range(w):
                z = depth_image[v, u]
                if z == 0:
                    continue
                x, y, z = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [u, v], z)
                points.append([x, y, z])
        points = np.array(points)
        return points

    def detect_plane(self, points):
        # Open3Dを使用して平面検出
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        plane_model, inliers = pc.segment_plane(distance_threshold=0.01,
                                                ransac_n=3,
                                                num_iterations=1000)
        if len(inliers) > 100:
            self.publish_plane_marker(plane_model, inliers, points)
            return plane_model, inliers
        else:
            self.get_logger().warning('平面が検出できませんでした。')
            return None, None

    def broadcast_tf(self, rotation, translation):
        # TransformStampedメッセージを作成
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_depth_optical_frame'

        # カメラの位置と姿勢を設定 (仮に位置を原点に設定)
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])

        # 回転をクォータニオン形式で設定
        quat = R.from_matrix(rotation).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # TFを送信
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = PlaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
