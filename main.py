"""
░░      ░░ ░░░░░░   ░░░░░  ░░░░░░      ░░      ░░░░░░   ░░░░░░   ░░░░░░
▒▒      ▒▒ ▒▒   ▒▒ ▒▒   ▒▒ ▒▒   ▒▒     ▒▒      ▒▒   ▒▒ ▒▒  ▒▒▒▒ ▒▒
▒▒      ▒▒ ▒▒   ▒▒ ▒▒▒▒▒▒▒ ▒▒▒▒▒▒      ▒▒      ▒▒   ▒▒ ▒▒ ▒▒ ▒▒ ▒▒▒▒▒▒▒
▓▓      ▓▓ ▓▓   ▓▓ ▓▓   ▓▓ ▓▓   ▓▓     ▓▓      ▓▓   ▓▓ ▓▓▓▓  ▓▓ ▓▓    ▓▓
███████ ██ ██████  ██   ██ ██   ██     ███████ ██████   ██████   ██████
"""

import serial
import matplotlib.pyplot as plt
import math
import numpy as np
import platform
import time
import os
import threading

from CalcLidarData import CalcLidarData


def save_thread(x_list, y_list, color):
    data = np.column_stack((x_list, y_list, color))

    # TODO: preserve dtype=np.int32 for color column
    # from  numpy.lib.recfunctions import append_fields
    # data = append_fields(data, 'color', color, usemask=False, dtypes=[np.int64])

    np.savetxt(f"{output_path}/{time.time()}.csv", data, delimiter=";")


# convert list to RGB-array (using 3x same value)
def color_array(l):
    a = np.array(l)

    # increase contrast by factor 2
    a = (a - 128) * 2
    a = a.clip(0, 255).astype(np.uint8)

    colors = np.column_stack((a, a, a))
    return colors


def init_visualisation(radial=False):
    # init lists for radial coordinates (LEGACY)
    angles = list()
    distances = list()

    fig = plt.figure(figsize=(8, 8))
    plotrange = 20

    if radial:
        ax = fig.add_subplot(111, projection='polar')
        ax.set_ylim([0, plotrange])

    else:
        ax = fig.add_subplot(111)
        ax.set_ylim([-plotrange, plotrange])
        ax.set_xlim([-plotrange, plotrange])

    ax.set_title('LiDAR LD06', fontsize=18)
    ax.set_facecolor('darkblue')
    ax.xaxis.grid(True, color='darkgray', linestyle='dashed')
    ax.yaxis.grid(True, color='darkgray', linestyle='dashed')

    return ax, angles, distances


# PARAMETERS
offset_angle = math.pi / 2  # 90° offset
visualisation = True        # use matplotlib live view
radial = False              # only used if visualisation is True
output_path = "csv"

# create output directory
if not os.path.exists(output_path):
    os.makedirs(output_path)

# select port by OS
ports = {'Windows': 'COM6',
         'Linux': '/dev/ttyUSB0'}  # '/dev/tty.usbserial-0001'
port = ports[platform.system()]

# init LD06 serial connection
serialConnection = serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)

# init lists for cartesian coordinates and confidence=color
x_list = list()
y_list = list()
confidence_list = list()


# VISUALISATION
if visualisation:
    ax, angles, distances = init_visualisation(radial)


"""
 █▄ ▄█ ▄▀▄ █ █▄ █   █   ▄▀▄ ▄▀▄ █▀▄
 █ ▀ █ █▀█ █ █ ▀█   █▄▄ ▀▄▀ ▀▄▀ █▀ 
"""
data_string = ""
i = 0
# threads = list()

while True:
    loopFlag = True
    flag2c = False

    if i % 40 == 39:
        if 'line' in locals():
            line.remove()

        # visualize either using radial or cartesian coordinates
        if visualisation:
            if radial:
                line = ax.scatter(angles, distances, c="blue", s=1)
                ax.set_theta_direction(-1)         # matplotlib polar default is CCW, so CW is -1
                ax.set_theta_offset(offset_angle)  # offset rotation so marking on LD06 is 0°
            else:
                rgb = color_array(confidence_list)
                line = ax.scatter(x_list, y_list, c=rgb/255, s=1)

        # update plot
        plt.pause(0.01)

        # save csv
        t = threading.Thread(target=save_thread, args=(x_list, y_list, confidence_list))
        t.start()

        # clear
        x_list.clear()
        y_list.clear()
        confidence_list.clear()
        # also clear optional radial coordinates
        if visualisation:
            if radial:
                angles.clear()
                distances.clear()

        i = 0

    while loopFlag:
        data_bytes = serialConnection.read()
        data_int = int.from_bytes(data_bytes, 'big')

        if data_int == 0x54:
            data_string += data_bytes.hex() + " "
            flag2c = True
            continue

        elif data_int == 0x2c and flag2c:
            data_string += data_bytes.hex()

            if not len(data_string[0:-5].replace(' ', '')) == 90:
                data_string = ""
                loopFlag = False
                flag2c = False
                continue

            lidarData = CalcLidarData(data_string[0:-5], offset_angle=offset_angle)

            x_list.extend(lidarData.x)
            y_list.extend(lidarData.y)
            confidence_list.extend(lidarData.Confidence_i)
            if visualisation:
                if radial:
                    angles.extend(lidarData.Angle_i)
                    distances.extend(lidarData.Distance_i)

            data_string = ""
            loopFlag = False
        else:
            data_string += data_bytes.hex() + " "

        flag2c = False

    i += 1

serialConnection.close()
