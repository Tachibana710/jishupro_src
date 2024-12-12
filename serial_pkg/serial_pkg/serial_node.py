import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String
from my_msgs.msg import SensorData,TargetCurrent

class SerialCanNode(Node):
    def __init__(self):
        super().__init__('serial_can_node')

        # Arduinoと接続するシリアルポートを指定
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 250000
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        time.sleep(2)  # 接続待機

        self.get_logger().info(f"Connected to Arduino on {self.serial_port}")

        # ROSトピックの購読と発行
        self.target_current_sub = self.create_subscription(
            TargetCurrent,
            '/target_current',
            self.send_command,
            1
        )
        self.feedback_pub = self.create_publisher(SensorData, '/sensor_data', 10)

        # ホールセンサ値取得用タイマー
        self.timer = self.create_timer(1, self.read_feedback)

        self.sensor_data = SensorData()

    def send_command(self, msg):
        try:
            # self.ser.write((msg.data + '\n').encode('utf-8'))
            # self.get_logger().info(f"Sent: {msg.data}")
            # if not isinstance(msg.target_current, list) or len(msg.target_current) < 3:
            #     self.get_logger().error("Invalid target_current: must be a list with at least 3 elements")
            #     return

        # 型変換（必要に応じて）
            target_current = [int(value) for value in msg.target_current]
            buf = bytearray()
            MSB1 = (int(msg.target_current[0]) >> 8) & 0xFF
            LSB1 = int(msg.target_current[0]) & 0xFF
            MSB2 = (int(msg.target_current[1]) >> 8) & 0xFF
            LSB2 = int(msg.target_current[1]) & 0xFF
            MSB3 = (int(msg.target_current[2]) >> 8) & 0xFF
            LSB3 = int(msg.target_current[2]) & 0xFF
            buf.append(0xAA)
            buf.append(MSB1)
            buf.append(LSB1)
            buf.append(MSB2)
            buf.append(LSB2)
            buf.append(MSB3)
            buf.append(LSB3)
            buf.append((0xAA + MSB1 + LSB1 + MSB2 + LSB2 + MSB3 + LSB3) & 0xFF)
            self.ser.write(buf)
            self.ser.flush()
            # self.get_logger().info(f"Sent raw: {buf}")
            # self.get_logger().info(f"Sent: {msg.target_current}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def read_print_output(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received: {data}")
        except Exception as e:
            return

    def read_feedback(self):
        self.get_logger().info("Reading feedback")
        target_current = [1000, 1000, 1000]
        buf = bytearray()
        MSB1 = (int(target_current[0]) >> 8) & 0xFF
        LSB1 = int(target_current[0]) & 0xFF
        MSB2 = (int(target_current[1]) >> 8) & 0xFF
        LSB2 = int(target_current[1]) & 0xFF
        MSB3 = (int(target_current[2]) >> 8) & 0xFF
        LSB3 = int(target_current[2]) & 0xFF
        buf.append(0xAA)
        buf.append(MSB1)
        buf.append(LSB1)
        buf.append(MSB2)
        buf.append(LSB2)
        buf.append(MSB3)
        buf.append(LSB3)
        buf.append((0xAA + MSB1 + LSB1 + MSB2 + LSB2 + MSB3 + LSB3) & 0xFF)
        self.ser.write(buf)
        # self.ser.flush()
        try:
            # if self.ser.in_waiting > 0:
            #     data = self.ser.read(size=8)
            #     motor_id = (int(data[6]) << 8 | data[7])

            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received: {data}")
                # data = self.ser.readline().decode('utf-8').strip()
                # self.get_logger().info(f"Received: {data}")
                # self.feedback_pub.publish(String(data=data))
        except Exception as e:
            return
            # self.get_logger().error(f"Failed to read feedback: {e}")
    def publish_message(self):
        """スライダーの値を取得してメッセージをパブリッシュ"""
        msg = TargetCurrent()
        msg.target_current = [slider.value() / 10.0 for slider in self.sliders]
        self.publisher.publish(msg)
        self.status_label.setText(f"Published: {msg.target_current}")

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
