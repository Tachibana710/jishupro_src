import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String

class SerialCanNode(Node):
    def __init__(self):
        super().__init__('serial_can_node')

        # Arduinoと接続するシリアルポートを指定
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        time.sleep(2)  # 接続待機

        self.get_logger().info(f"Connected to Arduino on {self.serial_port}")

        # ROSトピックの購読と発行
        self.command_sub = self.create_subscription(
            String,
            '/can_command',
            self.send_command,
            10
        )
        self.feedback_pub = self.create_publisher(String, '/can_feedback', 10)

        # ホールセンサ値取得用タイマー
        self.timer = self.create_timer(0.1, self.read_feedback)

    def send_command(self, msg):
        try:
            self.ser.write((msg.data + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def read_feedback(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received: {data}")
                self.feedback_pub.publish(String(data=data))
        except Exception as e:
            self.get_logger().error(f"Failed to read feedback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
