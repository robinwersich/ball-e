#!/usr/bin/env python

import threading
import sys
from collections import defaultdict, deque
from serial import Serial, SerialException
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

port = '/dev/ttyACM0'

try:
    ser = Serial(port, 115200)
except SerialException:
    print(f"Serial connection to {port} failed.")
    exit(1)


def read_user_input():
    while True:
        user_input = input() + '\n'
        ser.write(user_input.encode())


# Start separate thread for user input so it doesn't block the drawing
threading.Thread(target=read_user_input, daemon=True).start()

MAX_DATA_POINTS = sys.argv[1] if len(sys.argv) > 1 else 1000
timestamps = defaultdict(lambda: deque(maxlen=MAX_DATA_POINTS))
values = defaultdict(lambda: deque(maxlen=MAX_DATA_POINTS))


def update(i):
    while ser.in_waiting > 0:
        line = ser.readline().decode().strip()
        if not line.startswith('$ '):
            print(line)
            continue
        data_id, timestamp, value = line.removeprefix('$ ').split(',')

        timestamps[data_id].append(float(timestamp))
        values[data_id].append(float(value))

    if not values:
        return
    
    plt.cla()
    for data_id in values.keys():
        plt.plot(timestamps[data_id], values[data_id], label=data_id)
    plt.legend(loc="upper left")
    plt.tight_layout()
    

animation = FuncAnimation(plt.gcf(), update, interval=50, save_count=100)
plt.show()

ser.close()
