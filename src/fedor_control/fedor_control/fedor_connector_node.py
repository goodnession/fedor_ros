import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .tcp_connector.tcpconnector import TcpConnector


PERIOD = 0.01  # Устанавливаем в секундах период отправки сообщений в топиках
HOST = '192.168.0.105'
PORT = 10099


class FedorConnector(Node):
    """Узел для работы с симулятором (делает запросы и получает ответ).

    Наследуется от (rclpy.node.Node).
    """

    def __init__(self):
        # Присваиваем ноду имя
        super().__init__('fedor_connector')
        # Подключаемся к симулятору
        self.conn = TcpConnector(
            host=HOST,
            port=PORT,
        )
        # Инициализируем publisher текущих позиций моторов
        self.publisher_motors_position = self.create_publisher(
            String,
            'motors_position_topic',
            10,
        )
        timer_period = PERIOD
        self.timer_pub_motors_position = self.create_timer(
            timer_period,
            self.pub_motors_position_callback,
        )
        # Инициализируем subscriber на список моторов
        self.subscription_motors_list = self.create_subscription(
            String,
            'motors_list_topic',
            self.sub_motors_list_callback,
            10,
        )
        self.motors_list = ''
        # Инициализируем subscriber на значения тока
        self.subscription_motors_velset = self.create_subscription(
            String,
            'motors_velset_topic',
            self.sub_motors_velset_callback,
            10,
        )

    def pub_motors_position_callback(self):
        """Отправлят текущие позиции моторов."""
        msg = String()
        motors_position = self.conn.request(
            f'robot:motors:{self.motors_list}:posget')
        msg.data = str(motors_position)
        # self.get_logger().info(msg.data)
        self.publisher_motors_position.publish(msg)

    def sub_motors_list_callback(self, msg):
        """Получает список моторов.

        Аргументы:

        msg -- полученное сообщение.
        """
        self.motors_list = msg.data

    def sub_motors_velset_callback(self, msg):
        temp = json.loads(msg.data)
        for el in temp:
            for k, v in el.items():
                self.conn.request(f'robot:motors:{k}:velset:{str(v)}')


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
