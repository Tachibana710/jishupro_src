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
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.0005)
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
        self.timer = self.create_timer(0.001, self.read_feedback)

        self.sensor_data = SensorData()
        self.target_current = [0,0,0]

        self.buffer = bytearray()

        self.rotation_num = [0,0,0]

    def send_command(self, msg):
        # try:
        #     # self.ser.write((msg.data + '\n').encode('utf-8'))
        #     # self.get_logger().info(f"Sent: {msg.data}")
        #     # if not isinstance(msg.target_current, list) or len(msg.target_current) < 3:
        #     #     self.get_logger().error("Invalid target_current: must be a list with at least 3 elements")
        #     #     return

        # # 型変換（必要に応じて）
        #     target_current = [int(value) for value in msg.target_current]
        #     buf = bytearray()
        #     MSB1 = (int(msg.target_current[0]) >> 8) & 0xFF
        #     LSB1 = int(msg.target_current[0]) & 0xFF
        #     MSB2 = (int(msg.target_current[1]) >> 8) & 0xFF
        #     LSB2 = int(msg.target_current[1]) & 0xFF
        #     MSB3 = (int(msg.target_current[2]) >> 8) & 0xFF
        #     LSB3 = int(msg.target_current[2]) & 0xFF
        #     buf.append(0xAA)
        #     buf.append(MSB1)
        #     buf.append(LSB1)
        #     buf.append(MSB2)
        #     buf.append(LSB2)
        #     buf.append(MSB3)
        #     buf.append(LSB3)
        #     buf.append((0xAA + MSB1 + LSB1 + MSB2 + LSB2 + MSB3 + LSB3) & 0xFF)
        #     self.ser.write(buf)
        #     self.ser.flush()
        #     # self.get_logger().info(f"Sent raw: {buf}")
        #     # self.get_logger().info(f"Sent: {msg.target_current}")
        # except Exception as e:
        #     self.get_logger().error(f"Failed to send command: {e}")
        print("send_command")
        print(msg.target_current)
        self.target_current = msg.target_current

    def read_print_output(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received: 0 {data[0]}, 1 {data[1]}, 2 {data[2]}, 3 {data[3]}, 4 {data[4]}, 5 {data[5]}, 6 {data[6]}, 7 {data[7]}")
        except Exception as e:
            return

    def read_feedback(self):
        # self.get_logger().info("Reading feedback")
        buf = bytearray()
        MSB1 = (int(self.target_current[0]) >> 8) & 0xFF
        LSB1 = int(self.target_current[0]) & 0xFF
        MSB2 = (int(self.target_current[1]) >> 8) & 0xFF
        LSB2 = int(self.target_current[1]) & 0xFF
        MSB3 = (int(self.target_current[2]) >> 8) & 0xFF
        LSB3 = int(self.target_current[2]) & 0xFF
        buf.append(0xAA)
        buf.append(MSB1)
        buf.append(LSB1)
        buf.append(MSB2)
        buf.append(LSB2)
        buf.append(MSB3)
        buf.append(LSB3)
        buf.append((0xAA + MSB1 + LSB1 + MSB2 + LSB2 + MSB3 + LSB3) & 0xFF)
        # self.get_logger().info(f"Sent raw: {buf}")
        self.ser.write(buf)
        # self.ser.flush()
        # try:
        #     # if self.ser.in_waiting > 0:
        #     #     data = self.ser.read(size=8)
        #     #     motor_id = (int(data[6]) << 8 | data[7])

        if self.ser.in_waiting > 0:
            # data = self.ser.readline()
            # if len(data) != 8:
                
            #     return
            # self.ser.reset_input_buffer()
            # self.ser.reset_output_buffer()
            data = self.ser.read(10)
            # data = [int(data[i]) for i in range(8)]

            # self.get_logger().info(f"Received: {data}")

            self.buffer += data
            if len(self.buffer) > 10:
                # self.get_logger().info(f"buffer: {len(self.buffer)}")
                found = False
                for i in range(len(self.buffer) - 10):
                    if self.buffer[i:i+3] == b'\xAA\xAA\xAA':
                        self.buffer = self.buffer[i:]
                        found = True
                        break
                if found:
                    received_data = self.buffer[:10]
                    # self.get_logger().info(f"Received: {received_data}")
                    self.buffer = self.buffer[10:]
                    id = received_data[3]
                    if id < 0 or id >= 4:
                        self.get_logger().error(f"Invalid motor ID: {id}")
                        return
                    received_angle = (received_data[4] << 8) | received_data[5]
                    if abs(received_angle) > 8192:
                        self.get_logger().error(f"Invalid angle: {received_angle}")
                        return
                    if self.sensor_data.angle_integ[id-1] is None:
                        self.sensor_data.angle_integ[id-1] = received_angle
                    else:
                        if received_angle - self.sensor_data.angle_raw[id-1] > 8192 / 2:
                            self.rotation_num[id-1] -= 1
                        elif received_angle - self.sensor_data.angle_raw[id-1] < -8192 / 2:
                            self.rotation_num[id-1] += 1
                        self.sensor_data.angle_integ[id-1] = received_angle + self.rotation_num[id-1] * 8192
                    self.sensor_data.angle_raw[id-1] = (received_data[4] << 8) | received_data[5]
                    self.sensor_data.rps_raw[id-1] = (received_data[6] << 8) | received_data[7]
                    self.sensor_data.actual_current[id-1] = (received_data[8] << 8) | received_data[9]
                   
                    self.feedback_pub.publish(self.sensor_data)
                else:
                    self.buffer = self.buffer[-15:]
                    self.get_logger().info(f"no data found")
                    return
            # data_length = 






            # self.get_logger().info(f"Received: {data[0]}, {data[1]}, {data[2]}, {data[3]}, {data[4]}, {data[5]}, {data[6]}, {data[7]}")
            # if data != [0 for _ in range(8)] and len(data) == 8:
            #     id = (data[6] << 8 | data[7])
            #     if id < 0 or id >= 4:
            #         # self.get_logger().error(f"Invalid motor ID: {id}")
            #         return
            #     # print("data7:",data[7]) 
            #     # print("data6:",data[6])
            #     # print("id:",id)
            #     self.sensor_data.angle_raw[id] = (data[0] << 8) | data[1]
            #     self.sensor_data.rps_raw[id] = (data[2] << 8) | data[3]
            #     self.sensor_data.actual_current[id] = (data[4] << 8) | data[5]
            #     self.feedback_pub.publish(self.sensor_data)
                # self.sensor_data.data
                # self.feedback_pub.publish(self.sensor_data)
            # data = self.ser.readline().decode('utf-8').strip()
            # self.get_logger().info(f"Received: {data}")
            # self.feedback_pub.publish(String(data=data))
        # except Exception as e:
        #     self.get_logger().error(f"Failed to read feedback: {e}")
        #     # return
        #     # self.get_logger().error(f"Failed to read feedback: {e}")
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
