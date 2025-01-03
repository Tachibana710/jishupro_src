import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import json
import cv2
import base64
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.current_image = None
        self.received_msg = None

        self.img_cnt = 0

    def listener_callback(self, msg):
        print('Image received')
        try:
            # Convert ROS2 Image message to OpenCV image
            self.received_msg = msg
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info('Image received and ready to save on key press.')
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def save_image(self):
        # if self.current_image is not None:
            try:

                encoded_data = base64.b64encode(self.received_msg.data).decode('utf-8')

                # Create JSON object
                image_data = {
                    'header': {
                        'stamp': {
                            'sec': self.received_msg.header.stamp.sec,
                            'nanosec': self.received_msg.header.stamp.nanosec
                        },
                        'frame_id': self.received_msg.header.frame_id
                    },
                    'height': self.received_msg.height,
                    'width': self.received_msg.width,
                    'encoding': self.received_msg.encoding,
                    'is_bigendian': self.received_msg.is_bigendian,
                    'step': self.received_msg.step,
                    'data': encoded_data
                }

                # Save JSON to file
                filename = 'dataset/image_{}.json'.format(self.img_cnt)
                self.img_cnt += 1
                with open(filename, 'w') as json_file:
                    json.dump(image_data, json_file, indent=4)

                self.get_logger().info(f'Image saved to {filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to save image: {e}')
        # else:
        #     self.get_logger().warning('No image available to save.')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            key = input("Press Enter to save the current image or Ctrl+C to exit: ")
            if key == '':
                node.save_image()
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Node encountered an error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
