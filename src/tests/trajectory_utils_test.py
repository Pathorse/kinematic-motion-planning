import numpy as np

from dynamics.astro import Astro

from utilities.trajectory_utilities import *

def test_linear_interpolation():

    start = np.array([0, 0, 0])

    goal = np.array([10, 10, 0])

    time_steps = 16

    x = linear_interpolation(start, goal, time_steps)

    print('x: ', x)

def test_foot_interpolation():

    robot = Astro()

    base_state = np.array([0, 0, 0.187, 0, 0, 0])
    joint_state = np.radians(np.array([45, -20, 90, -45, 20, -90, 135, 20, -90, -135, -20, 90]))
    q = np.concatenate((base_state, joint_state))

    desired_net_distance = np.array([1, 0, 0])

    rxy_fl, rxy_fr, rxy_rl, rxy_rr = footpoint_xy_interpolation(robot, q, desired_net_distance, 16)

    print('Front left footpoint: ', rxy_fl)
    print('Front right footpoint: ', rxy_fr)
    print('Rear left footpoint: ', rxy_rl)
    print('Rear right footpoint: ', rxy_rr)