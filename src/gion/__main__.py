#!/usr/bin/env python3
import argparse
import time

import numpy as np
from pionfunc.functions import get_local_ip
from rich import inspect
from swarm_server import SwarmCommunicator

from gion import Gion

params = {
    "kp": np.array([[1.2, 1.2, 1, 1, 1, 1]]) * 0.15,
    "ki": np.zeros((1, 6)),
    "kd": np.array([[1, 1, 1, 1, 1, 1]]) * 2,
    "attraction_weight": 1.0,
    "cohesion_weight": 1.0,
    "current_velocity_weight": 8,
    "alignment_weight": 1.0,
    "repulsion_weight": 20.0,
    "unstable_weight": 1.0,
    "noise_weight": 1.0,
    "safety_radius": 0.4,
    "max_acceleration": 0.5,
    "max_speed": 0.3,
    "unstable_radius": 1.5,
}


def main():
    ip = get_local_ip()
    for i in range(15):
        if ip != "127.0.0.1":
            break
        ip = get_local_ip()
    parser = argparse.ArgumentParser(
        description="Запуск универсального сервера для управления единицей роя"
    )
    parser.add_argument(
        "--ip",
        type=str,
        default=ip,
        help="ip устройства (например, 10.1.100.121)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5656,
        help="Порт устройства, например 5656",
    )
    args = parser.parse_args()

    geobot = Gion(
        ip=args.ip,
        # mavlink_port=args.port,
        connection_method="udpout",
        name=f"Geobot-{args.ip}",
        dt=0.001,
        logger=True,
        max_speed=0.5,
    )
    inspect(geobot, methods=True)

    swarm_comm = SwarmCommunicator(
        control_object=geobot,
        broadcast_port=37020,
        broadcast_interval=0.5,
        ip=args.ip,
        time_sleep_update_velocity=0.1,
        params=params,
    )
    swarm_comm.start()
    print(f"SwarmCommunicator запущен для {geobot.name} с IP {args.ip}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        swarm_comm.stop()
        print("Swarm communicator остановлен.")


if __name__ == "__main__":
    main()
