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

rotation
https://www.kite.com/python/answers/how-to-rotate-a-3d-vector-about-an-axis-in-python
https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
"""

import numpy as np
from scipy.spatial.transform import Rotation
import os
import open3d as o3d


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


def listfiles(dir, extension=None):
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

    points3d = rotate3d(points3d, rotation_axis, angle)
    pointcloud = np.append(pointcloud, points3d, axis=0)

    angle += angular_resolution

print("pointcloud:", pointcloud.shape)
plot_3D(pointcloud)
