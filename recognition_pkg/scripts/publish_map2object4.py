#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
# import tf_transformations
import transforms3d.quaternions as tf_quaternions
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import std_msgs.msg
import numpy as np

class TransformRepublisher(Node):
    def __init__(self):
        super().__init__('transform_republisher')

        # Initialize TransformBroadcaster
        # self.br = TransformBroadcaster(self)
        self.observ_y_pub = self.create_publisher(std_msgs.msg.Float32, 'recognition/observ_y', 10)

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically compute and publish the transform
        self.timer = self.create_timer(0.1, self.publish_map_to_object4)
        self.get_logger().info("TransformRepublisher is running...")

    def publish_map_to_object4(self):
        try:
            # Look up the transform from map to camera
            map_to_camera = self.tf_buffer.lookup_transform(
                'map', 'camera_depth_optical_frame', rclpy.time.Time())

            # Look up the transform from camera to object4
            camera_to_object4 = self.tf_buffer.lookup_transform(
                'camera_depth_optical_frame', 'object4', rclpy.time.Time())

            # Combine the two transforms to get map to object4
            # map_to_object4 = self.combine_transforms(map_to_camera, camera_to_object4)

            map_to_object4 = self.tf_buffer.lookup_transform(
                'map', 'object4', rclpy.time.Time())

            # Publish the map to object4 transform
            # self.br.sendTransform(map_to_object4)

            self.observ_y_pub.publish(std_msgs.msg.Float32(data=map_to_object4.transform.translation.y - 0.065))
            # self.get_logger().info(f"Published transform from map to object4 at time {map_to_object4.header.stamp.sec}.{map_to_object4.header.stamp.nanosec}")

        except Exception as e:
            self.get_logger().warn(f"Could not compute transform: {e}")

    def combine_transforms(self, t1, t2):
        # Combine translations
        t1_translation = np.array([t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z])
        t2_translation = np.array([t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z])

        # Combine rotations using quaternions
        t1_rotation = [t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w]
        t2_rotation = [t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w]

        combined_translation = t1_translation + tf_quaternions.rotate_vector(t2_translation, t1_rotation)
        combined_rotation = tf_quaternions.qmult(t1_rotation, t2_rotation)

        # Create a new TransformStamped message for the combined transform
        combined = TransformStamped()
        combined.header.stamp = self.get_clock().now().to_msg()
        combined.header.frame_id = t1.header.frame_id
        combined.child_frame_id = t2.child_frame_id

        combined.transform.translation.x = combined_translation[0]
        combined.transform.translation.y = combined_translation[1]
        combined.transform.translation.z = combined_translation[2]
        combined.transform.rotation.x = combined_rotation[0]
        combined.transform.rotation.y = combined_rotation[1]
        combined.transform.rotation.z = combined_rotation[2]
        combined.transform.rotation.w = combined_rotation[3]

        return combined

def main(args=None):
    rclpy.init(args=args)

    # Create the TransformRepublisher node
    node = TransformRepublisher()

    try:
        # Spin the node to keep it running
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        # Shutdown the node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
