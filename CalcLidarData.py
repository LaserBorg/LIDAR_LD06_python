# import binascii
import math
import numpy as np

# creates object for return-values of *CalcLidarData* function
class LidarData:
    def __init__(self, FSA, LSA, CS, Speed, TimeStamp, Confidence_i, Angle_i, Distance_i):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.Speed = Speed
        self.TimeStamp = TimeStamp

        self.Confidence_i = Confidence_i
        self.Angle_i = Angle_i
        self.Distance_i = Distance_i

        # self.Position = polar2cartesian(Angle_i, Distance_i)


# # https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
def polar2cartesian(angle, distance):
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)
    return x, y
# def cartesian2polar(x, y):
#     distance = np.sqrt(x**2 + y**2)
#     angle = np.arctan2(y, x)
#     return angle, distance


def CalcLidarData(string):
    string = string.replace(' ', '')

    Speed = int(string[2:4] + string[0:2], 16) / 100
    FSA = float(int(string[6:8] + string[4:6], 16)) / 100         # Start Angle
    LSA = float(int(string[-8:-6] + string[-10:-8], 16)) / 100    # End Angle
    TimeStamp = int(string[-4:-2] + string[-6:-4], 16)
    CS = int(string[-2:], 16)                                  # Check Code

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

    lidarData = LidarData(FSA, LSA, CS, Speed, TimeStamp, Confidence_i, Angle_i, Distance_i)
    return lidarData
