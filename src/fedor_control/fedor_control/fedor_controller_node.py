import json
import requests

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from .motor.motor import Motor


PERIOD = 0.01  # Устанавливаем в секундах период отправки сообщений в топиках


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
        self.publisher_motors_list = self.create_publisher(
            String,
            'motors_list_topic',
            10,
        )
        timer_period = PERIOD
        self.timer_pub_motors_list = self.create_timer(
            timer_period,
            self.pub_motors_list_callback,
        )
        # Инициализируем publisher значений тока
        self.publisher_motors_control_signal = self.create_publisher(
            String,
            'motors_torqset_topic',
            10,
        )
        self.timer_pub_motors_control_signal = self.create_timer(
            timer_period,
            self.pub_motors_control_signal_callback,
        )
        # Инициализируем моторы
        # Считываем из конфига моторов
        self.motors = []
        with open(
                'install/fedor_control/share/ament_index/resource_index/packages/motors.yaml',
                'r',
        ) as stream:
            try:
                m = yaml.safe_load(stream)

                for el in m:
                    self.motors.append(
                        Motor(
                            name=el['name'],
                            minAngPos=el['minAngPos'],
                            maxAngPos=el['maxAngPos'],
                            vel=el['vel'],
                            torq=el['torq'],
                            kP=el['kP'],
                            kI=el['kI'],
                            kD=el['kD'],
                            t=PERIOD,
                            description=el['description'],
                        ),
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

            for i in range(len(self.motors)):
                self.motors[i].current_position = round(float(motors_data[i]), 2)

            mot = []

            for mt in self.motors:
                mot.append(
                    dict([
                        ('name', mt.name),
                        ('description', mt.description),
                        ('current_position', mt.current_position),
                        ('minAngPos', mt.minAngPos),
                        ('maxAngPos', mt.maxAngPos),
                    ]),
                )

            requests.post(
                'http://127.0.0.1:5000/motors_position',
                json=json.dumps(mot),
            )


    def pub_motors_list_callback(self) -> None:
        """Отправляет список моторов."""
        msg = String()

        for i in range(len(self.motors)):
            msg.data += self.motors[i].name + ';'

        self.publisher_motors_list.publish(msg)

    def pub_motors_control_signal_callback(self) -> None:
        """Вычисляет с помощью ПИД-регулятора и отправляет значение тока мотороам."""
        motors_pid = requests.get('http://127.0.0.1:5000/motors_pid').json()
        # Устанавливаем ПИД-коэффициенты
        for el in motors_pid:
            for i in range(len(self.motors)):
                if el['name'] == self.motors[i].name:
                    if el['kP'] != '':
                        self.motors[i].kP = float(el['kP'])
                    if el['kI'] != '':
                        self.motors[i].kI = float(el['kI'])
                    if el['kD'] != '':
                        self.motors[i].kD = float(el['kD'])

        motors_setpoint = requests.get('http://127.0.0.1:5000/motors_setpoint').json()
        temp = []

        for el in motors_setpoint:
            for i in range(len(self.motors)):
                if el['name'] == self.motors[i].name and el['setpoint'] != '':
                    temp.append(
                        dict([
                            (self.motors[i].name, self.motors[i].pid_compute(el['setpoint'])),
                        ]),
                    )

        msg = String()
        msg.data = json.dumps(temp)
        self.publisher_motors_control_signal.publish(msg)


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
