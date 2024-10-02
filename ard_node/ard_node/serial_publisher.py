import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')
        self.serial_port = '/dev/ttyUSB1'  # 아두이노와 연결된 시리얼 포트를 지정하세요
        self.serial_baudrate = 9600  # 아두이노와의 시리얼 통신 속도를 지정하세요
        self.serial_connection = serial.Serial(self.serial_port, self.serial_baudrate)
        self.subscription = self.create_subscription(
            String, 'keyboard_input', self.serial_callback, 10)
        self.get_logger().info('Serial Publisher Node is running.')

    def serial_callback(self, msg):
        user_input = msg.data
        self.get_logger().info(f'Sending message to Arduino: {user_input}')
        self.serial_connection.write(user_input.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

