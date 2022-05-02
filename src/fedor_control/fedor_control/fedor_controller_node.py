import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FedorController(Node):

    def __init__(self):
        super().__init__('fedor_controller')
        self.subscription = self.create_subscription(
            String,
            'motors_data_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    fedor_controller = FedorController()

    rclpy.spin(fedor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fedor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
