import rclpy
from rclpy.node import Node

import json
import serial
from serial import SerialException

from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

import queue
import threading
import logging
import time
import math

# ==== НАСТРОЙКА ПОРТА ПО УМОЛЧАНИЮ ====
serial_port = "/dev/ttyUSB0"


# ================= ВСПОМОГАТЕЛЬНЫЕ КЛАССЫ =================

class ReadLine:
    def __init__(self, s: serial.Serial):
        self.buf = bytearray()
        self.s = s

    def readline(self) -> bytes:
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r

        while True:
            i = max(1, min(512, self.s.in_waiting or 1))
            data = self.s.read(i)
            j = data.find(b"\n")
            if j >= 0:
                r = self.buf + data[:j + 1]
                self.buf[0:] = data[j + 1:]
                return r
            else:
                self.buf.extend(data)

    def clear_buffer(self):
        self.s.reset_input_buffer()


class BaseController:
    """
    Контроллер, работающий поверх УЖЕ ОТКРЫТОГО serial.Serial.
    Здесь же — очередь команд и чтение JSON-ответов (в т.ч. torque).
    """

    def __init__(self, ser: serial.Serial):
        self.logger = logging.getLogger('BaseController')

        # используем уже открытый порт из RoarmDriver
        self.ser = ser
        self.rl = ReadLine(self.ser)

        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(
            target=self.process_commands, daemon=True
        )
        self.command_thread.start()

        self.data_buffer = None
        self.base_data = {
            "T": 1051,
            "x": 0, "y": 0, "z": 0,
            "b": 0, "s": 0, "e": 0, "t": 0,
            "torB": 0, "torS": 0, "torE": 0, "torH": 0,
        }

    def feedback_data(self):
        """
        Читает одну строку JSON с UART и обновляет base_data.
        Ожидается, что там может быть T=1051 и поля torB/torS/torE/torH.
        """
        try:
            line = self.rl.readline().decode('utf-8').strip()
            if not line:
                return None
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            return self.base_data
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")
            self.rl.clear_buffer()
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()
        return None

    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))
        return data_read

    def send_command(self, data: dict):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            try:
                self.ser.write((json.dumps(data) + '\n').encode("utf-8"))
            except Exception as e:
                self.logger.error(f"[base_ctrl.process_commands] write error: {e}")

    def base_json_ctrl(self, input_json: dict):
        self.send_command(input_json)


# ================= ГЛАВНЫЙ ROS2-НОД =================

class RoarmDriver(Node):

    def __init__(self):
        super().__init__('roarm_driver')

        self.declare_parameter('serial_port', serial_port)
        self.declare_parameter('baud_rate', 115200)

        serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            # основной serial-порт
            self.serial_port = serial.Serial(serial_port_name, baud_rate, timeout=1)
            self.get_logger().info(f"Opened serial port {serial_port_name} at {baud_rate} baud.")

            # стартовый пакет (как в исходнике Waveshare)
            start_data = json.dumps({'T': 605, "cmd": 0}) + "\n"
            self.serial_port.write(start_data.encode())
            time.sleep(0.1)

        except SerialException as e:
            self.get_logger().error(f"Failed to open {serial_port_name}: {e}")
            self.serial_port = None
            return

        # контроллер поверх того же serial (для torque/feedback)
        self.base_controller = BaseController(self.serial_port)

        # === ПОДПИСКИ ===
        self.joint_states_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10
        )
        self.pose_sub = self.create_subscription(
            Pose, 'hand_pose', self.pose_callback, 10
        )
        self.led_ctrl_sub = self.create_subscription(
            Float32, 'led_ctrl', self.led_ctrl_callback, 10
        )

        # === ПАБЛИШЕР ДЛЯ TORQUE ===
        self.torque_pub = self.create_publisher(
            Float32MultiArray,
            'servo_torque',
            10
        )

        # === ТАЙМЕР ОПРОСА FEEDBACK (в т.ч. torque) ===
        # 10 Гц — можно менять период при необходимости
        self.feedback_timer = self.create_timer(
            0.1,
            self.feedback_timer_callback
        )

    # ---------- управление по joint_states (как в оригинале) ----------

    def joint_states_callback(self, msg: JointState):
        try:
            name = msg.name
            position = msg.position

            base = -position[name.index('base_link_to_link1')]
            shoulder = -position[name.index('link1_to_link2')]
            elbow = position[name.index('link2_to_link3')]
            hand = math.pi - position[name.index('link3_to_gripper_link')]

            data = json.dumps({
                'T': 102,
                'base': base,
                'shoulder': shoulder,
                'elbow': elbow,
                'hand': hand,
                'spd': 0,
                'acc': 10
            }) + "\n"

            self.serial_port.write(data.encode())
            time.sleep(0.05)

        except ValueError as e:
            self.get_logger().error(f"Joint name missing in joint_states: {e}")
        except SerialException as e:
            self.get_logger().error(f"SerialException in joint_states_callback: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in joint_states_callback: {e}")

    # ---------- callback по hand_pose (можно оставить пустым) ----------

    def pose_callback(self, msg: Pose):
        # В исходном коде здесь просто запрашивали feedback и логировали.
        # Теперь опрос и torque делает feedback_timer_callback, поэтому этот
        # callback можно не использовать или оставить под будущее.
        pass

    # ---------- таймерный опрос feedback + публикация torque ----------

    def feedback_timer_callback(self):
        """
        Периодически:
          1) шлём команду T:105 (запрос позы/torque);
          2) читаем JSON-ответ;
          3) если T == 1051 — публикуем torque в /servo_torque.
        """
        try:
            # отправляем запрос на feedback
            self.base_controller.base_json_ctrl({'T': 105})
            # маленькая пауза, чтобы контроллер успел ответить
            time.sleep(0.02)

            feedback = self.base_controller.feedback_data()
            if not feedback:
                return

            if feedback.get("T") != 1051:
                return

            # конвертация мм → м для x/y/z (если нужны)
            for key in ("x", "y", "z"):
                if key in feedback:
                    try:
                        feedback[key] = float(feedback[key]) / 1000.0
                    except Exception:
                        pass

            # формируем сообщение torque
            torque_msg = Float32MultiArray()
            torque_msg.data = [
                float(feedback.get("torB", 0.0)),
                float(feedback.get("torS", 0.0)),
                float(feedback.get("torE", 0.0)),
                float(feedback.get("torH", 0.0)),
            ]

            self.torque_pub.publish(torque_msg)

            # можно раскомментировать для дебага:
            # self.get_logger().info(f"Tq [B,S,E,H] = {torque_msg.data}")

        except Exception as e:
            self.get_logger().error(f'Error in feedback_timer_callback: {e}')

    # ---------- управление подсветкой ----------

    def led_ctrl_callback(self, msg: Float32):
        try:
            led_ctrl_data = json.dumps({
                'T': 114,
                "led": msg.data,
            }) + "\n"
            self.serial_port.write(led_ctrl_data.encode())
        except SerialException as e:
            self.get_logger().error(f"SerialException in led_ctrl_callback: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in led_ctrl_callback: {e}")


# ================= ТОЧКА ВХОДА =================

def main(args=None):
    rclpy.init(args=args)

    roarm_driver = RoarmDriver()

    if roarm_driver.serial_port is not None and roarm_driver.serial_port.is_open:
        rclpy.spin(roarm_driver)
        roarm_driver.destroy_node()
        roarm_driver.serial_port.close()
    else:
        # если порт не открылся — не спиним ноду
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
