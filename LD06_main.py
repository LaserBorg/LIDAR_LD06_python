"""
LD06 LiDAR data visualisation and saving to csv

communication protocol specified here:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf

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
    def __init__(self, FSA, LSA, CS, speed, timeStamp, luminance_list, angle_list, distance_list, offset):
        # raw sensor data (polar coordinates):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.speed = speed
        self.timeStamp = timeStamp
        self.luminance_list = luminance_list
        self.angle_list = angle_list
        self.distance_list = distance_list
        self.offset = offset
        
        # compute cartesian coordinates
        self.x, self.y = self.polar2cartesian(self.angle_list, self.distance_list, self.offset)
    
    # https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
    @staticmethod
    def polar2cartesian(angle, distance, offset):
        angle = list(np.array(angle) + offset)
        x = distance * -np.cos(angle)
        y = distance * np.sin(angle)
        return x, y
    
    # @staticmethod
    # def cartesian2polar(x, y):
        distance = np.sqrt(x**2 + y**2)
        angle = np.arctan2(y, x)
        return angle, distance

    @staticmethod
    def compute(string, offset=0):
        string = string.replace(' ', '')

        # convert hex string at indices 2 and 3 to an integer and perform bitwise AND to get last 5 bits.
        # TODO: protocol returns 14 altough 12 is definitely correct. why?
        dlength = 12 # int(string[2:4], 16) & 0x1F  

        speed = int(string[2:4] + string[0:2], 16) / 100            # rotational speed in degrees/second
        FSA = float(int(string[6:8] + string[4:6], 16)) / 100       # start angle in degrees
        LSA = float(int(string[-8:-6] + string[-10:-8], 16)) / 100  # end angle in degrees
        timestamp = int(string[-4:-2] + string[-6:-4], 16)          # timestamp in milliseconds
        CS = int(string[-2:], 16)                                   # CRC Checksum                                
        

        angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)
        
        angle_list = list()
        distance_list = list()
        luminance_list = list()

        counter = 0
        circle = lambda deg: deg - 360 if deg >= 360 else deg
        for i in range(0, 6 * 12, 6):

            angle = circle(angleStep * counter + FSA) * math.pi / 180.0
            angle_list.append(angle)

            distance = int(string[8 + i + 2:8 + i + 4] + string[8 + i:8 + i + 2], 16) / 100
            distance_list.append(distance)

            luminance = int(string[8 + i + 4:8 + i + 6], 16)
            luminance_list.append(luminance)

            counter += 1

        return LD06_data(FSA, LSA, CS, speed, timestamp, luminance_list, angle_list, distance_list, offset)
    
    @staticmethod
    def compute_bytes(data_bytes, offset=0):
        dlength = 12 

        # rotational speed in degrees/second
        speed = int.from_bytes(data_bytes[2:4], 'big') / 100
        
        # start angle in degrees
        FSA = float(int.from_bytes(data_bytes[4:6], 'big')) / 100       
        
        # end angle in degrees
        LSA = float(int.from_bytes(data_bytes[-6:-4], 'big')) / 100  
        
        # timestamp in milliseconds
        timestamp = int.from_bytes(data_bytes[-4:-2], 'big')          
        
        # CRC Checksum
        CS = data_bytes[-1]
        

        angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)
        
        angle_list = list()
        distance_list = list()
        luminance_list = list()

        counter = 0
        circle = lambda deg: deg - 360 if deg >= 360 else deg
        for i in range(0, 3 * dlength, 3):

            angle = circle(angleStep * counter + FSA) * math.pi / 180.0
            angle_list.append(angle)

            distance = int.from_bytes(data_bytes[6 + i:8 + i], 'big') / 100
            distance_list.append(distance)

            luminance = data_bytes[8 + i]
            luminance_list.append(luminance)

            counter += 1
        
        return LD06_data(FSA, LSA, CS, speed, timestamp, luminance_list, angle_list, distance_list, offset)


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

        # convert grayscale value to RGB-array
        def color_array(luminance):
            a = np.array(luminance)
            return np.column_stack((a, a, a))

        def animate(i):
            if hasattr(self, 'line'):
                self.line.remove()

            rgb = color_array(self.luminance_list)
            self.line = self.ax.scatter(self.x_list, self.y_list, c=rgb/255, s=1)
            return self.line,

        self.ani = animation.FuncAnimation(self.fig, animate, init_func=init, frames=1, interval=1, blit=True)
        return self.ani
    
    def update_data(self, x_list, y_list, luminance_list):
        self.x_list = x_list
        self.y_list = y_list
        self.luminance_list = luminance_list
        
        # pause to update plot
        plt.pause(self.pause)


def save_thread(x_list, y_list, color):
    data = list(zip(x_list, y_list, color))
    with open(f"{output_path}/{time.time()}.csv", 'w', newline='') as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerows(data)


# PARAMETERS
angle_offset = math.pi / 2  # 90°
visualize = True 
save_csv = True
output_path = "csv"
update_interval = 40


# create output directory
if not os.path.exists(output_path):
    os.makedirs(output_path)

# init output lists
x_list = list()
y_list = list()
luminance_list = list()

if visualize:
    visualisation = LD06_visualisation_2D()

# SERIAL CONNECTION
with LD06_serialConnection() as serial_connection:
    byte_string = ""
    #data_bytes = b""
    i = 0
    while True:
        # quit by pressing 'q'
        if keyboard.is_pressed('q'):
            break

        # save data to csv, visualise data and clear lists
        if i % update_interval == update_interval - 1:
            if visualize:
                visualisation.update_data(x_list, y_list, luminance_list)

            if save_csv:
                t = threading.Thread(target=save_thread, args=(x_list, y_list, luminance_list))
                t.start()

            # prepare for next iteration
            x_list.clear()
            y_list.clear()
            luminance_list.clear()
            i = 0

        # iterate through serial stream until start or end of package is found
        flag_2c = False
        while True:
            data_byte = serial_connection.read()
            data_int = int.from_bytes(data_byte, 'big')

            if data_int == 0x54:  # start of package
                byte_string += data_byte.hex() + " "
                flag_2c = True
                continue

            elif data_int == 0x2c and flag_2c: # end of package
                byte_string += data_byte.hex()

                # if len(byte_string[0:-6].replace(' ', '')) != 90:
                if len(byte_string) != 140:  # 90 byte + spaces + 0x54 + 0x2c
                    byte_string = ""
                    flag_2c = False
                    continue
                
                # crop last two byte (0x54, 0x2c) from byte_string
                lidar_data = LD06_data.compute(byte_string[0:-5], angle_offset)

                x_list.extend(lidar_data.x)
                y_list.extend(lidar_data.y)
                luminance_list.extend(lidar_data.luminance_list)

                byte_string = ""
                break

            else:
                byte_string += data_byte.hex() + " "

            flag_2c = False

        i += 1





""" 
* How to rotate a 3D vector about an axis in Python *

Rotate a vector v about axis by taking the component of v perpendicular to axis,
rotating it theta in the plane perpendicular to axis, then add the component of v parallel to axis.

Let a be a unit vector along an axis axis. Then a = axis/norm(axis).
Let A = I x a, the cross product of a with an identity matrix I.
Then exp(theta,A) is the rotation matrix.
Finally, dotting the rotation matrix with the vector will rotate the vector.

rotation
https://www.kite.com/python/answers/how-to-rotate-a-3d-vector-about-an-axis-in-python
https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
"""

from scipy.spatial.transform import Rotation
import open3d as o3d


def rotate3d(points3d, rotation_axis, rotation_degrees):
    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation_matrix = Rotation.from_rotvec(rotation_vector).as_matrix()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points3d[:, 0:3])
    pcd.rotate(rotation_matrix, center=(0, 0, 0))

    result_points = np.asarray(pcd.points)
    if points3d.shape[1] == 4:
        # reattach intensity column
        result_points = np.column_stack((result_points, points3d[:, 3]))

    return result_points


def plot_3D(pointcloud):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
    
    if pointcloud.shape[1] == 4:
        # normalize intensity values to [0, 1]
        intensities = pointcloud[:, 3] / np.max(pointcloud[:, 3])

        # map intensities to a grayscale color map
        colors = np.column_stack([intensities, intensities, intensities])
        pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd])


def listfiles(dir, extension=None):
    filepaths = list()
    for root, dirs, files in os.walk(dir, topdown=False):
        for file in files:
            if extension is None or os.path.splitext(file)[1] == extension:
                filepath = os.path.join(root, file)
                filepaths.append(filepath)
    return filepaths


# 3D conversion
rotation_axis = np.array([0, 0, 1])  # 3D rotation about up-axis: Z
angular_resolution = 1  # in degrees

# init result object with (X,y,Z, intensity)
pointcloud = np.zeros((1, 4))
angle = 0

filepaths = listfiles("csv", extension=".csv")

for filepath in filepaths:
    points2d = np.loadtxt(filepath, delimiter=";")

    # insert 3D Y=0 after column 0 so 2D Y becomes 3D Z (Z-up: image is now vertical)
    points3d = np.insert(points2d, 1, values=0, axis=1)

    points3d = rotate3d(points3d, rotation_axis, angle)
    pointcloud = np.append(pointcloud, points3d, axis=0)

    angle += angular_resolution

plot_3D(pointcloud)
