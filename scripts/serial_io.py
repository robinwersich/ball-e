#!/usr/bin/env python

import threading
import argparse
from collections import defaultdict, deque
from serial import Serial, SerialException
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


class Plotter:
    def __init__(self, serial: Serial, max_data_points: int):
        self.serial = serial
        self.timestamps = defaultdict(lambda: deque(maxlen=max_data_points))
        self.values = defaultdict(lambda: deque(maxlen=max_data_points))
        self.animation = FuncAnimation(plt.gcf(), self.update, interval=50, save_count=100)

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
        self.vectors = {}

    def update(self, i):
        while self.serial.in_waiting > 0:
            line = self.serial.readline().decode().strip()
            if not line.startswith('$v '):
                print(line)
                continue
            data_id, x, y, z = line.removeprefix('$v ').split(',')
            self.vectors[data_id] = (float(x), float(y), float(z))
        
        if not self.vectors:
            return
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for data_id, (x, y, z) in self.vectors.items():
            ax.quiver(0, 0, 0, x, y, z, label=data_id)
        ax.legend()
        plt.tight_layout()

    def show(self):
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Serial IO')
    parser.add_argument('-p', '--port', type=str, help='Serial port', default='/dev/ttyACM0')
    parser.add_argument('-b', '--baudrate', type=int, help='Baudrate', default=115200)
    parser.add_argument('-d', '--data-points', type=int, help='Max data points', default=1000)
    parser.add_argument('-m', '--mode', type=str, help='Display data over time or 3D vectors', choices=['plot', '3D'], default='plot')
    args = parser.parse_args()

    try:
        ser = Serial(args.port, args.baudrate)
    except SerialException:
        print(f"Serial connection to {args.port} failed.")
        exit(1)

    def read_user_input():
        while True:
            user_input = input() + '\n'
            ser.write(user_input.encode())

    try:
        threading.Thread(target=read_user_input, daemon=True).start()
        if args.mode == 'plot':
            Plotter(ser, args.data_points).show()
        elif args.mode == '3D':
            VectorViewer(ser).show()
    finally:
        ser.close()


if __name__ == '__main__':
    main()
