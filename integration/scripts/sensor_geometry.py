"""Sensor-only geometry: no navigation pose correction or benchmark scoring."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def pose_twist(previous, current):
    """Pose samples are (position, quaternion, seconds); output is body-frame twist."""
    dt = current[2] - previous[2]
    if dt <= 0:
        raise ValueError('non-increasing pose stamp')
    rotation = Rotation.from_quat(current[1])
    linear = rotation.inv().apply((np.asarray(current[0])-previous[0])/dt)
    angular = (Rotation.from_quat(previous[1]).inv()*rotation).as_rotvec()/dt
    return linear, angular


def sensor_position(position, quaternion, chassis_offset):
    return np.asarray(position) + Rotation.from_quat(quaternion).apply(chassis_offset)


def world_specific_force(acceleration, quaternion):
    return Rotation.from_quat(quaternion).inv().apply(
        np.asarray(acceleration) + [0., 0., 9.80665])


def inside_self(base, lower, upper):
    return ((base >= lower) & (base <= upper)).all(axis=1)


def self_returns(xyz, mount):
    pitch = -mount['pitch_up_rad']
    c, s = math.cos(pitch), math.sin(pitch)
    rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    base = xyz @ rotation.T + mount['position_m']
    return inside_self(base, mount['self_filter_min_m'], mount['self_filter_max_m'])
