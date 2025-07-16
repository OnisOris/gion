from typing import Optional

import numpy as np
from pion import Pion, SwarmCommunicator
from pionfunc.functions import (
    update_array,
    update_vector,
)
from pymavlink import mavutil


class SwarmCommunicatorGion(SwarmCommunicator):
    pass


class Gion(Pion):
    def arm(self) -> None:
        raise NotImplementedError(
            f"{__class__.__name__} can't implement this function."
        )

    def disarm(self) -> None:
        raise NotImplementedError(
            f"{__class__.__name__} can't implement this function."
        )

    def takeoff(self) -> None:
        raise NotImplementedError(
            f"{__class__.__name__} can't implement this function."
        )

    def land(self) -> None:
        raise NotImplementedError(
            f"{__class__.__name__} can't implement this function."
        )

    def _process_message(
        self, msg, src_component: Optional[int] = None
    ) -> None:
        """
        Обрабатывает одно сообщение и обновляет данные (позиция, ориентация, батарея)

        :param msg: Сообщение MAVLink
        :param src_component: Источник данных, по которому фильтруется сообщение.
        :return: None
        """
        if msg.get_type() == "LOCAL_POSITION_NED":
            self.position = np.array(
                [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz]
            )
            self.last_points = update_array(
                self.last_points, self.position[0:3]
            )
        elif msg.get_type() == "ATTITUDE":
            self.attitude = np.array(
                [
                    0,
                    0,
                    msg.yaw - np.pi / 2,
                    0,
                    0,
                    msg.yawspeed,
                ]
            )
            self.last_angles = update_vector(self.last_angles, self.yaw)
        elif msg.get_type() == "BATTERY_STATUS":
            self.battery_voltage = msg.voltages[0] / 100

    def set_speed_for_wheels(
        self, left_wheel_speed: int, right_wheel_speed: int
    ) -> None:
        """
        Задание скорости для двух колес

        :param left_wheel_speed: скорость левого колеса, от -255 до 255
        :param right_wheel_speed: скорость правого колеса, от -255 до 255
        """
        self.mavlink_socket.mav.rc_channels_override_send(
            self.mavlink_socket.target_system,
            self.mavlink_socket.target_component,
            255,  # channel 1
            255,  # channel 2
            255,  # channel 3
            255,  # channel 4
            right_wheel_speed + 255,  # channel 5
            -left_wheel_speed + 255,  # channel 6
            255,  # channel 7
            255,  # channel 8
        )

    def send_speed(
        self, vx: float, vy: float, vz: float, yaw_rate: float
    ) -> None:
        """
        Метод задает вектор скорости дрону. Отсылать необходимо в цикле.

        :param vx: скорость по оси x (м/с)
        :type vx: float
        :param vy: скорость по оси y (м/с)
        :type vy: float
        :param vz:  скорость по оси z (м/с)
        :type vz: float
        :param yaw_rate:  скорость поворота по оси z (рад/с)
        :type yaw_rate: float
        :return: None
        """
        return self._send_command_long(
            command_name="ABS_SPEED",
            command=mavutil.mavlink.MAV_CMD_USER_5,
            param1=vx,
            param2=vy,
            target_system=self.mavlink_socket.target_system,
            target_component=self.mavlink_socket.target_component,
        )

    def compute_wheels(self, vx, vy, theta, r, b, omega_max):
        """
        Пересчёт вектора скорости (vx, vy) в целевые значения для колёс.
        """
        # 1. Ротация в локальную систему
        vx_r = np.cos(theta) * vx + np.sin(theta) * vy
        vy_r = -np.sin(theta) * vx + np.cos(theta) * vy

        # 2. Продольная и угловая скорости
        V = vx_r
        L = b / 2
        omega = vy_r / L

        # 3. Обратная кинематика
        omega_R = (V + omega * L) / r
        omega_L = (V - omega * L) / r

        # 4. Шкалирование в PWM
        pwm_R = max(-255, min(255, int((omega_R / omega_max) * 255)))
        pwm_L = max(-255, min(255, int((omega_L / omega_max) * 255)))
        print(f"l = {pwm_L}, r = {pwm_R}")
        return pwm_L, pwm_R

    def led_control(self, r=0, g=0, b=0, led_id=255):  # 255 all led
        max_value = 255.0
        all_led = 255
        first_led = 0
        last_led = 3
        led_value = [r, g, b]
        command = True

        try:
            if led_id != all_led and (led_id < first_led or led_id > last_led):
                command = False
            for i in range(len(led_value)):
                led_value[i] = float(led_value[i])
                if led_value[i] > max_value or led_value[i] < 0:
                    command = False
                    break
        except ValueError:
            command = False
        if command:
            self._send_command_long(
                f"{self.name} led_control",  # command_name
                mavutil.mavlink.MAV_CMD_USER_1,  # command
                param1=led_id,  # param1
                param2=led_value[0],  # param2
                param3=led_value[1],  # param3
                param4=led_value[2],  # param4
                target_system=self.mavlink_socket.target_system,
                target_component=self.mavlink_socket.target_component,
            )
        else:
            print(f"{self.name} <LED> Wrong LED values")

    def led_custom(self, mode=1, timer=0, color1=(0, 0, 0), color2=(0, 0, 0)):
        param2 = (((color1[0] << 8) | color1[1]) << 8) | color1[2]
        param3 = (((color2[0] << 8) | color2[1]) << 8) | color2[2]

        self._send_command_long(
            f"{self.name} led_custom",  # command name
            mavutil.mavlink.MAV_CMD_USER_3,  # command
            0,  # param1
            param2,  # param2
            param3,  # param3
            0,  # param4
            param5=mode,  # param5
            param6=timer,  # param6
            target_system=self.mavlink_socket.target_system,
            target_component=self.mavlink_socket.target_component,
        )

    def stop_moving(self) -> None:
        """
        Останавливает все движение

        :return: None
        """
        print("STOP_MOVING")
        self.speed_flag = False
        self.rc_flag = False
        self.tracking = False
        self.point_reached = True
        self.send_speed(0, 0, 0, 0)
