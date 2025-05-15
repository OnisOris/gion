from typing import Optional

import numpy as np
from pion import Pion
from pionfunc.functions import (
    update_array,
    update_vector,
)


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
        # Проверяем источник компонента, если задан
        if self.checking_components:
            if (
                src_component is not None
                and msg._header.srcComponent != src_component
            ):
                return
        if msg.get_type() == "LOCAL_POSITION_NED":
            if self.dimension == 3:
                self.position = np.array(
                    [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz]
                )
            else:
                self.position = np.array([msg.x, msg.y, msg.vx, msg.vy])
            self.last_points = update_array(
                self.last_points, self.position[0 : self.dimension]
            )
        elif msg.get_type() == "ATTITUDE":
            self.attitude = np.array(
                [
                    0,
                    0,
                    msg.yaw,
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
        self,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        """
        Метод задает вектор скорости геоботу. Отсылать необходимо в цикле.

        :param vx: скорость по оси x (м/с)
        :type vx: float
        :param vy: скорость по оси y (м/с)
        :type vy: float
        :param vz:  скорость по оси z (м/с)
        :type vz: float
        :return: None
        """
        w_l, w_r = self.compute_wheels(
            vx, vy, self.attitude[2], 0.07, 0.1, 255
        )
        self.set_speed_for_wheels(w_l, w_r)

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
