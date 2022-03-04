import serial
import matplotlib.pyplot as plt
import math
from CalcLidarData import CalcLidarData

###############
# VISUALISATION

COLOR = 'darkgray'
plt.rcParams['text.color'] = COLOR
plt.rcParams['axes.labelcolor'] = COLOR
plt.rcParams['xtick.color'] = COLOR
plt.rcParams['ytick.color'] = COLOR

fig = plt.figure(facecolor='white', figsize=(8, 8))
ax = fig.add_subplot(111, projection='polar')
ax.set_title('LiDAR LD06', fontsize=18)
ax.set_facecolor('white')
ax.set_ylim([0, 20])
ax.xaxis.grid(True, color='lightgray', linestyle='dashed')
ax.yaxis.grid(True, color='lightgray', linestyle='dashed')

# press 'e' to quit
plt.connect('key_press_event', lambda event: exit(1) if event.key == 'q' else None)
###############


serialConnection = serial.Serial(port="COM6",  # '/dev/ttyUSB0',
                                 baudrate=230400,
                                 timeout=5.0,
                                 bytesize=8,
                                 parity='N',
                                 stopbits=1)

data_string = ""
lines = list()
angles = list()
distances = list()

cartesian = list()

i = 0
while True:
    loopFlag = True
    flag2c = False

    if i % 40 == 39:
        if 'line' in locals():
            line.remove()
        line = ax.scatter(angles, distances, c="blue", s=5)

        ax.set_theta_offset(math.pi / 2)
        plt.pause(0.01)
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

            lidarData = CalcLidarData(data_string[0:-5])
            angles.extend(lidarData.Angle_i)
            distances.extend(lidarData.Distance_i)

            data_string = ""
            loopFlag = False
        else:
            data_string += data_bytes.hex() + " "

        flag2c = False

    i += 1

serialConnection.close()
