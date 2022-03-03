"""
* How to rotate a 3D vector about an axis in Python *

Rotate a vector v about axis by taking the component of v perpendicular to axis,
rotating it theta in the plane perpendicular to axis, then add the component of v parallel to axis.

Let a be a unit vector along an axis axis. Then a = axis/norm(axis).
Let A = I x a, the cross product of a with an identity matrix I.
Then exp(theta,A) is the rotation matrix.
Finally, dotting the rotation matrix with the vector will rotate the vector.

reference:
https://www.kite.com/python/answers/how-to-rotate-a-3d-vector-about-an-axis-in-python
https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
"""

import numpy as np
from scipy.spatial.transform import Rotation


def rotate3d(points3d, rotation_axis, rotation_degrees):
    # define 3D rotation
    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation = Rotation.from_rotvec(rotation_vector)

    # apply rotation to each point
    for i, point in enumerate(points3d):
        points3d[i] = rotation.apply(point)

    return points3d


# image is Y-up
points2d = [[-20.0, 16.],
            [-19.5, 17.],
            [-19.0, 18.],
            [20.0, -19.]]

# rotating about Y-axis
rotation_axis = np.array([0, 1, 0])
rotation_degrees = 10

# append Z dimension to get 3D points
zero_column = np.zeros((4, 1), dtype=float)
points3d = np.append(points2d, zero_column, axis=1)

# rotate
points3d = rotate3d(points3d, rotation_axis, rotation_degrees)
print(points3d)
