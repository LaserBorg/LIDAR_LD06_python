import serial
import matplotlib.pyplot as plt
import math
import platform
from CalcLidarData import CalcLidarData

###############
# # VISUALISATION

radial = False
offset_angle = math.pi / 2

# COLOR = 'darkgray'
# plt.rcParams['text.color'] = COLOR
# plt.rcParams['axes.labelcolor'] = COLOR
# plt.rcParams['xtick.color'] = COLOR
# plt.rcParams['ytick.color'] = COLOR

fig = plt.figure(figsize=(8, 8))
range =20

if radial is True:
    ax = fig.add_subplot(111, projection='polar')
    ax.set_ylim([0, range])
else:
    ax = fig.add_subplot(111)
    ax.set_ylim([-range, range])
    ax.set_xlim([-range, range])

ax.set_title('LiDAR LD06', fontsize=18)


# ax.set_facecolor('white')
# ax.xaxis.grid(True, color='lightgray', linestyle='dashed')
# ax.yaxis.grid(True, color='lightgray', linestyle='dashed')

# # press 'q' to quit
# plt.connect('key_press_event', lambda event: exit(1) if event.key == 'q' else None)
###############

# select port by OS
ports = {'Windows': 'COM6',
         'Linux': '/dev/ttyUSB0'}  # '/dev/tty.usbserial-0001'
port = ports[platform.system()]

serialConnection = serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)

if radial is True:
    angles = list()
    distances = list()
else:
    x_list = list()
    y_list = list()

data_string = ""
i = 0
while True:
    loopFlag = True
    flag2c = False

    if i % 40 == 39:
        if 'line' in locals():
            line.remove()

        if radial is True:
            line = ax.scatter(angles, distances, c="blue", s=2)

            # matplotlib polar default is CCW, so CW is -1
            ax.set_theta_direction(-1)

            # offset rotation so marking on LD06 is 0°
            ax.set_theta_offset(offset_angle)

        else:
            line = ax.scatter(x_list, y_list, c="blue", s=2)

        # update plot
        plt.pause(0.01)

        if radial is True:
            angles.clear()
            distances.clear()
        else:
            x_list.clear()
            y_list.clear()

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

            if radial is True:
                angles.extend(lidarData.Angle_i)
                distances.extend(lidarData.Distance_i)
            else:
                x_list.extend(lidarData.x)
                y_list.extend(lidarData.y)

            data_string = ""
            loopFlag = False
        else:
            data_string += data_bytes.hex() + " "

        flag2c = False

    i += 1

serialConnection.close()
