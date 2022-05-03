import json

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from .motor.motor import Motor

PERIOD = 0.5  # Устанавливаем в секундах период отправки сообщений в топиках


class FedorController(Node):
    """Узел, управляющий моторами.

    Наследуется от (rclpy.node.Node).
    """

    def __init__(self) -> None:
        # Присваиваем ноду имя
        super().__init__('fedor_controller')
        # Инициализируем subscriber на текущую позицию моторов
        self.subscription_motors_position = self.create_subscription(
            String,
            'motors_position_topic',
            self.sub_motors_position_callback,
            10,
        )
        # Инициализируем publisher списка моторов
        self.publisher_motors_list = self.create_publisher(String,
                                                           'motors_list_topic',
                                                           10,
                                                           )
        timer_period = PERIOD
        self.timer_pub_motors_list = self.create_timer(timer_period,
                                                       self.pub_motors_list_callback,
                                                       )
        # Инициализируем моторы
        # Считываем из конфига моторов
        self.motors = []
        with open('install/fedor_control/share/ament_index/resource_index/packages/motors.yaml',
                  'r'
                  ) as stream:
            try:
                m = yaml.safe_load(stream)
                for el in m:
                    self.motors.append(
                        Motor(name=el['name'],
                              minAngPos=el['minAngPos'],
                              maxAngPos=el['maxAngPos'],
                              torq=el['torq'],
                              kP=el['kP'],
                              kI=el['kI'],
                              kD=el['kD'],
                              t=PERIOD,
                              )
                    )
            except yaml.YAMLError as exc:
                self.get_logger().error(exc)

    def sub_motors_position_callback(self, msg) -> None:
        """Получает текущее положение моторов.

        Аргументы:

        msg -- полученное сообщение.
        """
        if msg.data != 'None':
            motors_data = msg.data.split(';')
            js = []
            for i in range(len(self.motors)):
                js.append(dict([(self.motors[i].name, motors_data[i])]))
            self.get_logger().info(json.dumps(js, indent=4))

    def pub_motors_list_callback(self) -> None:
        """Отправляет список моторов."""
        msg = String()
        for i in range(len(self.motors)):
            msg.data += self.motors[i].name + ';'
        self.get_logger().info(msg.data)
        self.publisher_motors_list.publish(msg)


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
