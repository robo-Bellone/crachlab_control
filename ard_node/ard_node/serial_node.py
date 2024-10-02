import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import String

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.ser = serial.Serial('/dev/ttyACM0', 9600)  # 시리얼 포트 설정

        # 키보드 입력을 위한 서브스크립션 생성
        self.keyboard_subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_input_callback,
            10)

        self.serial_thread = threading.Thread(target=self.read_from_port)
        self.serial_thread.start()

    def read_from_port(self):
        while True:
            reading = self.ser.readline().decode()
            if reading:
                self.get_logger().info('Received: ' + reading)

    def keyboard_input_callback(self, msg):
        user_input = msg.data
        self.get_logger().info(f'Sending message to Arduino: {user_input}')
        self.ser.write(user_input.encode())  # 시리얼 포트로 데이터 전송

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

