"""
LD06 LiDAR data visualisation and saving to csv
"""

import serial
import math
import numpy as np
import platform
import time
import os
import threading
import csv

import matplotlib.pyplot as plt
import matplotlib.animation as animation


def LD06_serial():
    # select port by OS
    ports = {'Windows': 'COM10',
            'Linux': '/dev/ttyUSB0'}  # '/dev/tty.usbserial-0001'
    port = ports[platform.system()]

    serialConnection = serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)
    return serialConnection


class LD06_data:
    def __init__(self, FSA, LSA, CS, Speed, TimeStamp, Confidence_i, Angle_i, Distance_i, offset=0):
        self.offset = offset
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.Speed = Speed
        self.TimeStamp = TimeStamp

        self.Confidence_i = Confidence_i
        self.Angle_i = Angle_i
        self.Distance_i = Distance_i

        x, y = self.polar2cartesian(self.Angle_i, self.Distance_i, self.offset)
        self.x = x
        self.y = y

    # https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
    @staticmethod
    def polar2cartesian(angle, distance, offset):
        angle = list(np.array(angle) + offset)
        x = distance * -np.cos(angle)
        y = distance * np.sin(angle)
        return x, y
    
    # def cartesian2polar(x, y):
    #     distance = np.sqrt(x**2 + y**2)
    #     angle = np.arctan2(y, x)
    #     return angle, distance

    @staticmethod
    def compute(string, offset=0):
        string = string.replace(' ', '')

        Speed = int(string[2:4] + string[0:2], 16) / 100
        FSA = float(int(string[6:8] + string[4:6], 16)) / 100         # Start Angle
        LSA = float(int(string[-8:-6] + string[-10:-8], 16)) / 100    # End Angle
        TimeStamp = int(string[-4:-2] + string[-6:-4], 16)
        CS = int(string[-2:], 16)                                     # Check Code

        Confidence_i = list()
        Angle_i = list()
        Distance_i = list()
        # count = 0  # unused variable?
        if LSA - FSA > 0:
            angleStep = float(LSA - FSA) / 12
        else:
            angleStep = float((LSA + 360) - FSA) / 12

        counter = 0
        circle = lambda deg: deg - 360 if deg >= 360 else deg
        for i in range(0, 6 * 12, 6):
            Distance_i.append(int(string[8 + i + 2:8 + i + 4] + string[8 + i:8 + i + 2], 16) / 100)
            Confidence_i.append(int(string[8 + i + 4:8 + i + 6], 16))
            Angle_i.append(circle(angleStep * counter + FSA) * math.pi / 180.0)
            counter += 1
        
        return LD06_data(FSA, LSA, CS, Speed, TimeStamp, Confidence_i, Angle_i, Distance_i, offset=offset)
    

def save_thread(x_list, y_list, color):
    # TODO: preserve dtype=np.int32 for color column
    # from  numpy.lib.recfunctions import append_fields
    # data = append_fields(data, 'color', color, usemask=False, dtypes=[np.int64])

    data = list(zip(x_list, y_list, color))
    with open(f"{output_path}/{time.time()}.csv", 'w', newline='') as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerows(data)


# convert list to RGB-array (using 3x same value)
def color_array(l):
    a = np.array(l)

    # increase contrast by factor 2
    a = (a - 128) * 2
    a = a.clip(0, 255).astype(np.uint8)

    colors = np.column_stack((a, a, a))
    return colors


def init_visualisation():
    def init():
        return ax,

    def animate(i):
        global x_list, y_list, confidence_list, line

        if 'line' in locals():
            line.remove()

        rgb = color_array(confidence_list)
        line = ax.scatter(x_list, y_list, c=rgb/255, s=1)
        return line,


    fig = plt.figure(figsize=(8, 8))
    plotrange = 20

    ax = fig.add_subplot(111)
    ax.set_ylim([-plotrange, plotrange])
    ax.set_xlim([-plotrange, plotrange])

    ax.set_title('LiDAR LD06', fontsize=18)
    ax.set_facecolor('darkblue')
    ax.xaxis.grid(True, color='darkgray', linestyle='dashed')
    ax.yaxis.grid(True, color='darkgray', linestyle='dashed')

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=1, interval=1, blit=True)

    return ani, ax



# PARAMETERS
offset = math.pi / 2  # 90° offset
visualize = True 
save_csv = True
output_path = "csv"


# create output directory
if not os.path.exists(output_path):
    os.makedirs(output_path)

# init lists for cartesian coordinates and confidence=color
x_list = list()
y_list = list()
confidence_list = list()

# VISUALISATION
if visualize:
    ani, ax = init_visualisation()


# SERIAL CONNECTION
serialConnection = LD06_serial()

"""
MAIN LOOP
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

        # visualize cartesian coordinates
        if visualize:
            rgb = color_array(confidence_list)
            line = ax.scatter(x_list, y_list, c=rgb/255, s=1)

        # update plot
        plt.pause(0.001)

        # save csv
        if save_csv:
            t = threading.Thread(target=save_thread, args=(x_list, y_list, confidence_list))
            t.start()

        # clear
        x_list.clear()
        y_list.clear()
        confidence_list.clear()

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
            
            lidarData = LD06_data.compute(data_string[0:-5], offset)

            x_list.extend(lidarData.x)
            y_list.extend(lidarData.y)
            confidence_list.extend(lidarData.Confidence_i)

            data_string = ""
            loopFlag = False
        else:
            data_string += data_bytes.hex() + " "

        flag2c = False

    i += 1

serialConnection.close()
