from typing import Any, Optional

import numpy as np
from pion import Pion
from pionfunc.functions import (
    get_local_ip,
    start_threading,
    update_array,
    update_vector,
)
from pymavlink import mavutil
from swarm_server import CMD, SwarmCommunicator


class SwarmCommunicatorGion(SwarmCommunicator):
    def process_incoming_state(self, state: Any) -> None:
        """
        Функция обработчик входящих сообщений

        :return: None
        :rtype: None
        """

        if self.mode == 3:
            self.control_object.t_speed *= 0.9
        if np.linalg.norm(self.control_object.t_speed) < 0.02:
            self.control_object.t_speed = np.zeros_like(self.control_object.t_speed)

        if state.target_id:
            if self.unique_id != int(state.target_id):
                return
        elif state.group_id:
            if state.group_id != self.group_id:
                return
        if hasattr(state, "command") and state.command != 0:
            command = CMD(state.command)

            if command == CMD.SET_SPEED:
                try:
                    vx, vy, vz, yaw_rate = state.data
                    self.control_object.send_speed(vx, vy, vz, yaw_rate)
                    print(
                        f"Команда set_speed выполнена: {vx}, {vy}, {vz}, {yaw_rate}"
                    )
                except Exception as e:
                    print("Ошибка при выполнении set_speed:", e)
            elif command == CMD.SET_GROUP:
                try:
                    new_group = int(state.data[0])
                    self.group_id = new_group
                    self.control_object.name = f"{get_local_ip()}, unid: {self.unique_id}, gr: {self.group_id}"
                    print(
                        f"Группа успешно изменена на: {new_group} для дрона с id {self.unique_id}"
                    )
                except Exception as e:
                    print("Ошибка при изменении группы:", e)
            elif command == CMD.GOTO:
                try:
                    x, y, z, yaw = state.data
                    if self.control_object.tracking:
                        print(f"Smart tracking to {x, y, z, yaw}")
                        self.control_object.target_point = np.array(
                            [x, y, z, yaw]
                        )
                    else:
                        self.stop_trp()
                        self.control_object.goto_from_outside(x, y, z, yaw)
                        print(f"Команда goto выполнена: {x}, {y}, {z}, {yaw}")
                except Exception as e:
                    print("Ошибка при выполнении goto:", e)
            elif command == CMD.TAKEOFF:
                self.stop_trp()
                try:
                    self.control_object.takeoff()
                except NotImplementedError:
                    print("У объекта нет takeoff")
                print("Команда takeoff выполнена")
            elif command == CMD.LAND:
                self.stop_trp()
                try:
                    self.control_object.land()
                except NotImplementedError:
                    print("У объекта нет land")
                print("Команда land выполнена")
            elif command == CMD.ARM:
                self.stop_trp()
                try:
                    self.control_object.arm()
                except NotImplementedError:
                    print("У объекта нет arm")
                print("Команда arm выполнена")
            elif command == CMD.DISARM:
                self.stop_trp()
                try:
                    self.control_object.disarm()
                except NotImplementedError:
                    print("У объекта нет disarm")
                print("Команда disarm выполнена")
            elif command == CMD.STOP:
                self.stop_trp()
                print("Команды на достижение позиций остановлены")
            elif command == CMD.SWARM_ON:
                try:
                    if self.control_object.tracking:
                        print("Режим слежения за точкой уже включен")
                    else:
                        self.control_object.tracking = True
                        start_threading(self.smart_point_tracking)
                except Exception as e:
                    print("Ошибка при выполнении smart_goto:", e)

            elif command == CMD.SMART_GOTO:
                try:
                    self.stop_trp()
                    x, y, z, yaw = state.data
                    self.control_object.threads.append(
                        start_threading(self.smart_goto, x, y, z, yaw)
                    )
                except Exception as e:
                    print("Ошибка при выполнении smart_goto:", e)
            elif command == CMD.LED:
                try:
                    led_id, r, g, b = state.data
                    self.control_object.led_control(led_id, r, g, b)
                    print(
                        f"LED control executed: led_id={led_id}, r={r}, g={g}, b={b}"
                    )
                except Exception as e:
                    print("Ошибка при выполнении LED control:", e)
            elif command == CMD.SAVE:
                self.save_data()
            elif command == CMD.SET_MOD:
                # +-------------+-----+-----------+-----------------+
                # |             | PID | REPULSION | UNSTABLE_VECTOR |
                # +=============+=====+===========+=================+
                # | SWARM_MODE  | 1   | 1         | 1               |
                # +-------------+-----+-----------+-----------------+
                # | MASTER_MODE | 1   | 0         | 0               |
                # +-------------+-----+-----------+-----------------+
                # | FLOW_MODE   | 0   | 1         | 0               |
                # +-------------+-----+-----------+-----------------+
                try:
                    self.mode = int(state.data[0])
                    if self.mode == 1:
                        print("Swarm mode set 1")
                        self.restore_params()
                    elif self.mode == 2:
                        print("Swarm mode set 2")
                        self.restore_params()
                        self.swarm_solver.repulsion_weight = 0
                    elif self.mode == 3:
                        print("Swarm mode set 3")
                        self.restore_params()
                        self.swarm_solver.kp = self.params["kp"] * 0
                        self.swarm_solver.ki = self.params["ki"] * 0
                        self.swarm_solver.kd = self.params["kd"] * 0
                        self.swarm_solver.unstable_weight = 0.0
                except Exception as e:
                    print("Ошибка при выполнении set_mode:", e)
            else:
                print("Получена неизвестная команда:", state.command)
        else:
            if hasattr(state, "id"):
                if not state.id == self.numeric_id:
                    self.env[state.id] = state
            elif hasattr(state, "ip"):
                self.env[state.ip] = state
            state = list(self.env.values())
            positions = []

            for drone_data in state:
                try:
                    if (
                        hasattr(drone_data, "data")
                        and len(drone_data.data) >= 7
                    ):
                        position_slice = drone_data.data[1:7]
                        if len(position_slice) == 6:
                            positions.append(position_slice)
                except Exception as e:
                    print(f"[WARN] Ошибка при обработке позиции дрона: {e}")

            try:
                self.env_state_matrix = np.vstack(
                    [self.control_object.position, *positions]
                )
            except Exception as e:
                print(f"[ERROR] Невозможно собрать env_state_matrix: {e}")


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
