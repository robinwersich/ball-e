#!/usr/bin/env python

import json
import threading
import argparse
from collections import defaultdict, deque
from matplotlib.quiver import Quiver
from serial import Serial, SerialException
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Plotter:
    def __init__(self, serial: Serial, max_data_points: int):
        self.serial = serial
        self.timestamps = defaultdict(lambda: deque(maxlen=max_data_points))
        self.values = defaultdict(lambda: deque(maxlen=max_data_points))
        self.animation = FuncAnimation(plt.gcf(), self.update, interval=50, cache_frame_data=False)

    def update(self, i):
        while self.serial.in_waiting > 0:
            line = self.serial.readline().decode().strip()
            if not line.startswith('$p '):
                print(line)
                continue
            data_id, timestamp, value = line.removeprefix('$p ').split(',')

            self.timestamps[data_id].append(float(timestamp))
            self.values[data_id].append(float(value))

        if not self.values:
            return
        
        plt.cla()
        for data_id in self.values.keys():
            plt.plot(self.timestamps[data_id], self.values[data_id], label=data_id)
        plt.legend(loc="upper left")
        plt.tight_layout()

    def show(self):
        plt.show()


class VectorViewer:
    def __init__(self, serial: Serial):
        self.serial = serial
        self.quivers: dict[str, Quiver] = {}
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='x', arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='y', arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='z', arrow_length_ratio=0.1)
        self.animation = FuncAnimation(self.fig, self.update, interval=20, cache_frame_data=False)

    @staticmethod
    def color_from_id(data_id: str) -> str:
        return '#' + hex(hash(data_id))[-6:]


    def update(self, i):
        while self.serial.in_waiting > 0:
            line = self.serial.readline().decode().strip()
            if not line.startswith('$v '):
                print(line)
                continue
            data_id, x, y, z = line.removeprefix('$v ').split(',')
            if data_id in self.quivers:
                self.quivers[data_id].remove()
            self.quivers[data_id] = self.ax.quiver(
                0, 0, 0, float(x), float(y), float(z), label=data_id, color=self.color_from_id(data_id)
            )
        self.ax.legend(loc="upper left")

    def show(self):
        plt.show()


def load_parameters(serial: Serial, param_file: str | None) -> dict[str, str]:
    if not param_file:
        return {}
    try:
        with open(param_file, 'r') as f:
            print(f"Reading parameters from {param_file}.")
            return json.load(f)
    except FileNotFoundError:
        print(f"Parameter file {param_file} does not exist yet, will create.")
        return {}


def main():
    parser = argparse.ArgumentParser(description='Serial IO')
    parser.add_argument('-p', '--port', type=str, help='Serial port', default='/dev/ttyACM0')
    parser.add_argument('-b', '--baudrate', type=int, help='Baudrate', default=115200)
    parser.add_argument('-d', '--data-points', type=int, help='Max data points', default=1000)
    parser.add_argument('-m', '--mode', type=str, help='Display data over time or 3D vectors', choices=['plot', '3D'], default='plot')
    parser.add_argument('-f', '--param-file', type=str, help='File for reading and writing parameters', required=False)
    args = parser.parse_args()

    try:
        ser = Serial(args.port, args.baudrate)
    except SerialException:
        print(f"Serial connection to {args.port} failed.")
        exit(1)

    parameters = load_parameters(ser, args.param_file)
    for key, value in parameters.items():
        ser.write(f"set {key} {value}\n".encode())

    stop_input = False
    def read_user_input():
        while not stop_input:
            user_input = input() + '\n'
            arguments = user_input.split()
            if arguments[0] == 'set' and len(arguments) == 3:
                parameters[arguments[1]] = arguments[2]
            ser.write(user_input.encode())

    try:
        threading.Thread(target=read_user_input, daemon=True).start()
        if args.mode == 'plot':
            Plotter(ser, args.data_points).show()
        elif args.mode == '3D':
            VectorViewer(ser).show()
    except KeyboardInterrupt:
        stop_input = True
    finally:
        ser.close()
        if args.param_file:
            print(f"\nWriting parameters to {args.param_file}.")
            with open(args.param_file, 'w') as f:
                json.dump(parameters, f)


if __name__ == '__main__':
    main()
