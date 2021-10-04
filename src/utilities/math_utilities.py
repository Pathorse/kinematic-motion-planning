import numpy as np
import pydrake
import pydrake.symbolic as sym

from pydrake import *


# ----------------------------------------------------
# Defines the rotation matrix about the x-axis
# ----------------------------------------------------
def RotationMatrixX(theta, type='np'):

    m = sym if type == 'sym' else np

    # Rotation matrix about x-axis
    R = np.array([
        [1, 0,            0            ],
        [0, m.cos(theta), -m.sin(theta)],
        [0, m.sin(theta), m.cos(theta) ]
    ])

    return R

# ----------------------------------------------------
# Defines the rotation matrix about the y-axis
# ----------------------------------------------------
def RotationMatrixY(theta, type='np'):

    m = sym if type == 'sym' else np

    # Rotation matrix about y-axis
    R = np.array([
        [m.cos(theta),  0, m.sin(theta)],
        [0,             1, 0           ],
        [-m.sin(theta), 0, m.cos(theta)]
    ])

    return R

# ----------------------------------------------------
# Defines the rotation matrix about the z-axis
# ----------------------------------------------------
def RotationMatrixZ(theta, type='np'):

    m = sym if type == 'sym' else np

    # Rotation matrix about z-axis
    R = np.array([
        [m.cos(theta), -m.sin(theta), 0],
        [m.sin(theta), m.cos(theta),  0],
        [0,            0,             1]
    ])

    return R

# ----------------------------------------------------
# Defines the ZYX Euler Angles
# ----------------------------------------------------
def EulerAnglesZYX(yaw, pitch, roll, type='np'):

    Rz = RotationMatrixZ(yaw, type)
    Ry = RotationMatrixY(pitch, type)
    Rx = RotationMatrixX(roll, type)

    return Rz.dot(Ry).dot(Rx)

# ----------------------------------------------------
# Defines the XYZ Euler Angles
# ----------------------------------------------------
def EulerAnglesXYZ(roll, pitch, yaw, type='np'):

    Rx = RotationMatrixX(roll, type)
    Ry = RotationMatrixY(pitch, type)
    Rz = RotationMatrixZ(yaw, type)

    return Rx.dot(Ry).dot(Rz)