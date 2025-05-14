from pion import Pion

import numpy as np
from typing import Annotated, Any, Optional, Tuple, Union

from pionfunc.functions import (
    create_connection,
    scalar_reached,
    start_threading,
    update_array,
    update_vector,
    vector_reached,
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

    def set_speed_for_wheels(self,
                  left_wheel_speed: int,
                  right_wheel_speed: int) -> None:
        """
        Задание скорости для двух колес

        :param left_wheel_speed: скорость левого колеса, от -255 до 255
        :param right_wheel_speed: скорость правого колеса, от -255 до 255
        """
        self.mavlink_socket.mav.rc_channels_override_send(
            self.mavlink_socket.target_system,
            self.mavlink_socket.target_component,
            255,    # channel 1
            255,  # channel 2
            255,    # channel 3
            255,    # channel 4
            right_wheel_speed + 255,  # channel 5
            -left_wheel_speed + 255,  # channel 6
            255,  # channel 7
            255   # channel 8
        )
