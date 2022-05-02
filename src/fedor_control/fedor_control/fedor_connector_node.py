import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .tcp_connector.tcpconnector import TcpConnector


PERIOD = 0.5  # seconds
HOST = '192.168.0.101'
PORT = 10099


class FedorConnector(Node):

    def __init__(self):
        super().__init__('fedor_connector')
        self.publisher_ = self.create_publisher(String, 'motors_data_topic', 10)
        timer_period = PERIOD  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.conn = TcpConnector(host=HOST, port=PORT)

    def timer_callback(self):
        msg = String()
        huy = self.conn.request(
            'robot:motors:R.ShoulderF;R.ShoulderS;L.ShoulderF;L.ShoulderS:posget')
        msg.data = huy
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    fedor_connector = FedorConnector()

    rclpy.spin(fedor_connector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fedor_connector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
