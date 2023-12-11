"""
LD06 LiDAR data visualisation and saving to csv

inspired by https://github.com/henjin0/Lidar_LD06_for_Arduino
TODO: check https://github.com/Paradoxdruid/pyLIDAR
"""

import platform
import serial
import math
import numpy as np
import time
import os
import threading
import csv
import keyboard

import matplotlib.pyplot as plt
import matplotlib.animation as animation


def LD06_serialConnection():
    port = {'Windows': 'COM10', 'Linux': '/dev/ttyUSB0'}[platform.system()]  # '/dev/tty.usbserial-0001'
    baudrate = 230400
    timeout = 5.0
    bytesize = 8
    parity = 'N'
    stopbits = 1
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout, bytesize=bytesize, parity=parity, stopbits=stopbits)


class LD06_data:
    def __init__(self, FSA, LSA, CS, speed, timeStamp, confidence_list, angle_list, distance_list, offset=0):
        # raw return data (polar coordinates):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.speed = speed
        self.timeStamp = timeStamp
        self.confidence_list = confidence_list
        self.angle_list = angle_list
        self.distance_list = distance_list
        self.offset = offset

        # computed data (cartesian coordinates):
        self.x, self.y = self.polar2cartesian(self.angle_list, self.distance_list, self.offset)
    
    # https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
    @staticmethod
    def polar2cartesian(angle, distance, offset):
        angle = list(np.array(angle) + offset)
        x = distance * -np.cos(angle)
        y = distance * np.sin(angle)
        return x, y
    
    @staticmethod
    def cartesian2polar(x, y):
        distance = np.sqrt(x**2 + y**2)
        angle = np.arctan2(y, x)
        return angle, distance

    @staticmethod
    def compute(string, offset=0):
        string = string.replace(' ', '')

        # TODO: dlength in string is 14 but 12 seems to be correct..?
        dlength = 12  # int(string[2:4], 16) & 0x1F

        speed = int(string[2:4] + string[0:2], 16) / 100
        FSA = float(int(string[6:8] + string[4:6], 16)) / 100         # Start Angle
        LSA = float(int(string[-8:-6] + string[-10:-8], 16)) / 100    # End Angle
        timestamp = int(string[-4:-2] + string[-6:-4], 16)
        CS = int(string[-2:], 16)                                     # CRC Checksum                                
        

        angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)
        
        confidence_list = list()
        angle_list = list()
        distance_list = list()

        counter = 0
        circle = lambda deg: deg - 360 if deg >= 360 else deg
        for i in range(0, 6 * 12, 6):

            angle = circle(angleStep * counter + FSA) * math.pi / 180.0
            angle_list.append(angle)

            distance = int(string[8 + i + 2:8 + i + 4] + string[8 + i:8 + i + 2], 16) / 100
            distance_list.append(distance)

            confidence = int(string[8 + i + 4:8 + i + 6], 16)
            confidence_list.append(confidence)

            counter += 1
        
        # print("speed:",speed, "FSA:", FSA, "LSA:", LSA, "timestamp:", timestamp)
        # print("angle:", angle, "distance:", distance, "confidence:", confidence, "\n")

        return LD06_data(FSA, LSA, CS, speed, timestamp, confidence_list, angle_list, distance_list, offset=offset)


class LD06_visualisation_2D:
    def __init__(self, pause=0.005):
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111)
        self.plotrange = 20
        self.ax.set_ylim([-self.plotrange, self.plotrange])
        self.ax.set_xlim([-self.plotrange, self.plotrange])
        self.ax.set_title('LiDAR LD06', fontsize=18)
        self.ax.set_facecolor('darkblue')
        self.ax.xaxis.grid(True, color='darkgray', linestyle='dashed')
        self.ax.yaxis.grid(True, color='darkgray', linestyle='dashed')
        self.ani = self.init_visualisation()
        self.pause = pause
    
    def init_visualisation(self):
        def init():
            return self.ax,

        # convert list to RGB-array (using 3x same value)
        def color_array(luminance):
            a = np.array(luminance)

            # increase contrast by factor 2
            a = (a - 128) * 2
            a = a.clip(0, 255).astype(np.uint8)

            colors = np.column_stack((a, a, a))
            return colors

        def animate(i):
            if hasattr(self, 'line'):
                self.line.remove()

            rgb = color_array(self.confidence_list)
            self.line = self.ax.scatter(self.x_list, self.y_list, c=rgb/255, s=1)
            return self.line,

        self.ani = animation.FuncAnimation(self.fig, animate, init_func=init, frames=1, interval=1, blit=True)
        return self.ani
    
    def update_data(self, x_list, y_list, confidence_list):
        self.x_list = x_list
        self.y_list = y_list
        self.confidence_list = confidence_list
        
        # update plot
        plt.pause(self.pause)


def save_thread(x_list, y_list, color):
    data = list(zip(x_list, y_list, color))
    with open(f"{output_path}/{time.time()}.csv", 'w', newline='') as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerows(data)


# PARAMETERS
offset = 0 # math.pi / 2  # 90° offset
visualize = True 
save_csv = True
output_path = "csv"
update_interval = 40


# create output directory
if not os.path.exists(output_path):
    os.makedirs(output_path)

# init lists for cartesian coordinates and confidence=color
x_list = list()
y_list = list()
confidence_list = list()

if visualize:
    visualisation = LD06_visualisation_2D()

# SERIAL CONNECTION
serialConnection = LD06_serialConnection()

data_string = ""
i = 0
while True:
    # quit by pressing 'q'
    if keyboard.is_pressed('q'):
        serialConnection.close()
        break

    loopFlag = True
    flag2c = False

    if i % update_interval == update_interval-1:
        if visualize:
            visualisation.update_data(x_list, y_list, confidence_list)

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
            confidence_list.extend(lidarData.confidence_list)

            data_string = ""
            loopFlag = False
        else:
            data_string += data_bytes.hex() + " "

        flag2c = False
    i += 1
