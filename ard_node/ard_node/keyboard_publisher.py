import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info('Keyboard Publisher Node is running.')

    def run(self):
        while True:
            user_input = input("메시지를 입력하세요: ")
            msg = String()
            msg.data = user_input
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

