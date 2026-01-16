#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
import math


class TorquePlotter(Node):
    def __init__(self):
        super().__init__('torque_plotter')

        # Подписка на топик с torque
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'servo_torque',
            self.torque_callback,
            10
        )

        # Время и значения torque
        self.t = []  # время в секундах
        self.joint_data = [[], [], [], []]  # [base, shoulder, elbow, hand]

        self.start_time = self.get_clock().now()
        self.max_points = 1000  # чтобы не раздувать память, берём только последние N точек

    def torque_callback(self, msg: Float32MultiArray):
        # Текущее время с начала записи
        now = self.get_clock().now()
        dt = (now - self.start_time).nanoseconds / 1e9
        self.t.append(dt)

        # На всякий случай заполняем значениями или NaN, если чего-то нет
        for i in range(4):
            if i < len(msg.data):
                v = float(msg.data[i])
            else:
                v = math.nan
            self.joint_data[i].append(v)

        # Ограничиваем длину
        if len(self.t) > self.max_points:
            self.t = self.t[-self.max_points:]
            for i in range(4):
                self.joint_data[i] = self.joint_data[i][-self.max_points:]


def main(args=None):
    rclpy.init(args=args)

    node = TorquePlotter()

    # === НАСТРОЙКА MATPLOTLIB ===
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title("RoArm M2S torque (real-time)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Torque (units from servo)")
    
    # 4 линии: base, shoulder, elbow, hand
    line_base, = ax.plot([], [], label='base')
    line_shoulder, = ax.plot([], [], label='shoulder')
    line_elbow, = ax.plot([], [], label='elbow')
    line_hand, = ax.plot([], [], label='hand')
    ax.legend(loc='upper right')

    try:
        while rclpy.ok():
            # один шаг ROS2
            rclpy.spin_once(node, timeout_sec=0.01)

            # если данные уже есть — обновляем график
            if node.t:
                line_base.set_data(node.t, node.joint_data[0])
                line_shoulder.set_data(node.t, node.joint_data[1])
                line_elbow.set_data(node.t, node.joint_data[2])
                line_hand.set_data(node.t, node.joint_data[3])

                ax.relim()
                ax.autoscale_view()

            plt.pause(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()