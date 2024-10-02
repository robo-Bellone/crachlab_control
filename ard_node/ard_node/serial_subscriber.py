import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialSubscriber(Node):

    def __init__(self):
        super().__init__('serial_subscriber')
        self.subscription = self.create_subscription(
            String,
            'serial_data',  # Replace with your topic name
            self.serial_data_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.ser = serial.Serial('/dev/ttyACM0', 9600)  # Replace with your Arduino's serial port and baud rate
        self.get_logger().info('Serial Subscriber Node is running.')

    def serial_data_callback(self, msg):
        serial_data = self.ser.readline().decode().strip()
        self.get_logger().info(f'Received serial data: {serial_data}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

