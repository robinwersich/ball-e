#!/usr/bin/env python

import argparse
from os import path
import time

from serial import Serial, SerialException


GYRO_COMMAND = "gyro\n".encode()
ACCEL_COMMAND = "accel\n".encode()
MAG_COMMAND = "mag\n".encode()


def main():
    parser = argparse.ArgumentParser(description='Serial IO')
    parser.add_argument('-p', '--port', type=str, help='Serial port', default='/dev/ttyACM0')
    parser.add_argument('-b', '--baudrate', type=int, help='Baudrate', default=115200)
    parser.add_argument('-o', '--output-dir', type=str, help='Output directory', default='.')
    args = parser.parse_args()

    if not path.isdir(args.output_dir):
        print(f"Output directory {args.output_dir} does not exist.")
        exit(1)

    try:
        ser = Serial(args.port, args.baudrate)
    except SerialException:
        print(f"Serial connection to {args.port} failed.")
        exit(1)

    input(
        "Let's calibrate the magnetometer.\n"
        "Press enter and then rotate the IMU around the z axis within the next 10 seconds."
    )

    min_x, max_x, min_y, max_y = 0, 0, 0, 0
    end = time.time() + 10
    while time.time() < end:
        ser.write(MAG_COMMAND)
        mag_data = ser.readline().decode().strip()
        x, y = mag_data.split()
        min_x = min(min_x, float(x))
        max_x = max(max_x, float(x))
        min_y = min(min_y, float(y))
        max_y = max(max_y, float(y))
    
    x_bias = (min_x + max_x) / 2
    y_bias = (min_y + max_y) / 2
    mag_bias = f"{x_bias} {y_bias}"
    mag_file = path.join(args.output_dir, 'mag_data.txt')
    with open(mag_file, 'w') as f:
        f.write(mag_bias)
    print(f"Magnetometer bias ({mag_bias}) written to {mag_file}")

    input("Let's calibrate the gyrosensor.\nPlace the IMU so that it is still and press enter.")
    ser.write(GYRO_COMMAND)
    gyro_bias = ser.readline().decode().strip()
    gyro_file = path.join(args.output_dir, 'gyro_bias.txt')
    with open(gyro_file, 'w') as f:
        f.write(gyro_bias)
    print(f"Gyro bias ({gyro_bias}) written to {gyro_file}")

    print(
        "Let's calibrate the accelerometer.\n"
        "Place the IMU in different orientations and press enter to measure.\n"
        "Enter 'done' to finish.",
        end=''
    )

    accel_data: list[str] = []
    while True:
        user_input = input()
        if user_input == 'done':
            break
        ser.write(ACCEL_COMMAND)
        accel_measurement = ser.readline().decode().strip()
        print(accel_measurement, end='')
        accel_data.append(accel_measurement)

    accel_file = path.join(args.output_dir, 'accel_data.txt')
    with open(accel_file, 'w') as f:
        f.write('\n'.join(accel_data))
    print(f"{len(accel_data)} accelerometer measurements written to {accel_file}")


if __name__ == '__main__':
    main()
