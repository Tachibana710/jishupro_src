import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from my_msgs.msg import RecognitionResult  # Import your custom message type

class ImageMarkerNode(Node):
    def __init__(self):
        super().__init__('image_marker_node')

        # Initialize subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.coord_sub = self.create_subscription(
            RecognitionResult,
            '/recognition_result',
            self.coord_callback,
            10
        )

        # Initialize variables to store coordinates
        self.output_x = None
        self.output_y = None

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

    def coord_callback(self, msg):
        self.output_x = msg.output_x  # Store x coordinate
        self.output_y = msg.output_y  # Store y coordinate
        self.get_logger().info(f"Received coordinates: ({self.output_x}, {self.output_y})")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # If coordinates are available, draw a marker
            if self.output_x is not None and self.output_y is not None:
                cv2.drawMarker(cv_image, (self.output_x, self.output_y), (0, 0, 255), markerType=cv2.MARKER_CROSS, thickness=2)

            # Display the image
            cv2.imshow('Image with Marker', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        rclpy.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()