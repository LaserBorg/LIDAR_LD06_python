"""
░░░░░░   ░░░░░░  ░░░░░░░░  ░░░░░  ░░░░░░░░ ░░░░░░░     ░░░░░░  ░░░░░░
▒▒   ▒▒ ▒▒    ▒▒    ▒▒    ▒▒   ▒▒    ▒▒    ▒▒               ▒▒ ▒▒   ▒▒
▒▒▒▒▒▒  ▒▒    ▒▒    ▒▒    ▒▒▒▒▒▒▒    ▒▒    ▒▒▒▒▒        ▒▒▒▒▒  ▒▒   ▒▒
▓▓   ▓▓ ▓▓    ▓▓    ▓▓    ▓▓   ▓▓    ▓▓    ▓▓               ▓▓ ▓▓   ▓▓
██   ██  ██████     ██    ██   ██    ██    ███████     ██████  ██████

* How to rotate a 3D vector about an axis in Python *

Rotate a vector v about axis by taking the component of v perpendicular to axis,
rotating it theta in the plane perpendicular to axis, then add the component of v parallel to axis.

Let a be a unit vector along an axis axis. Then a = axis/norm(axis).
Let A = I x a, the cross product of a with an identity matrix I.
Then exp(theta,A) is the rotation matrix.
Finally, dotting the rotation matrix with the vector will rotate the vector.


references:

rotation
https://www.kite.com/python/answers/how-to-rotate-a-3d-vector-about-an-axis-in-python
https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html

matplotlib 3D
https://stackoverflow.com/questions/62433465/how-to-plot-3d-point-clouds-from-an-npy-file
https://stackoverflow.com/questions/8130823/set-matplotlib-3d-plot-aspect-ratio

"""

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import os


def plot_3D(pointcloud):
    xs = pointcloud[:, 0]
    ys = pointcloud[:, 1]
    zs = pointcloud[:, 2]
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(projection='3d')
    ax.set_box_aspect((np.ptp(xs), np.ptp(ys), np.ptp(zs)))
    img = ax.scatter(xs, ys, zs, s=1)  # , c=t_low, cmap=plt.hot())
    fig.colorbar(img)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


def rotate3d(points3d, rotation_axis, rotation_degrees):
    # define 3D rotation
    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation = Rotation.from_rotvec(rotation_vector)

    result_points = points3d.copy()

    if points3d.shape[1] == 4:
        # remove intensity column
        points3d = np.delete(points3d, -1, axis=1)
    
    # apply rotation to each point
    for i, point in enumerate(points3d):
        result_points[i][0:3] = rotation.apply(point)

    return result_points


def listfiles(dir, extension=None):
    # index all csv files
    filepaths = list()
    for root, dirs, files in os.walk(dir, topdown=False):
        for file in files:
            if extension is None or os.path.splitext(file)[1] == extension:
                filepath = os.path.join(root, file)
                filepaths.append(filepath)
    return filepaths


rotation_axis = np.array([0, 0, 1])  # 3D rotation about up-axis (Z)
angular_resolution = 1  # in degrees

# init result object with (X,y,Z, intensity)
pointcloud = np.zeros((1, 4))
angle = 0

filepaths = listfiles("csv", extension=".csv")

for filepath in filepaths:
    points2d = np.loadtxt(filepath, delimiter=";")

    # insert 3D Y=0 after column 0 so 2D Y becomes 3D Z (Z-up: image is now vertical)
    points3d = np.insert(points2d, 1, values=0, axis=1)

    # for angle in range(0, 360, angular_resolution):
    points3d = rotate3d(points3d, rotation_axis, angle)
    pointcloud = np.append(pointcloud, points3d, axis=0)

    angle += angular_resolution

print("pointcloud:", pointcloud.shape)
plot_3D(pointcloud)