import sys
import time

import numpy as np

from gion import Gion

np.set_printoptions(suppress=True)  # Отключить экспоненциальный формат
args = sys.argv
number_drone = sys.argv[1]
drone = Gion(
    ip=f"10.1.100.{number_drone}",
    mavlink_port=5656,
    logger=True,
    dt=0.0,
    count_of_checking_points=5,
)
drone.period_send_speed = 0.1

if "-c" in args:
    drone.led_control(255, 0, 255, 0)
    drone.logger = True
    while True:
        if drone._msg is not None:
            if drone._msg.get_type() == "ATTITUDE":
                print(drone._msg.yaw)
                print(f"yaw getter: {drone.yaw}")
        else:
            print("None")
        time.sleep(0.02)
elif "-r" in args:
    drone.reboot_board()
elif "-r" in args:
    drone.start_sound()
else:
    print("---")
    drone.send_speed(float(args[2]), float(args[3]), 0, 0)
