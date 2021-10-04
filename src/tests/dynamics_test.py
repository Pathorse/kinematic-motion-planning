import numpy as np

from dynamics.astro import Astro

def test_forward_kinematics():

    robot = Astro()

    base_state = np.array([0, 0, 0.187, 0, 0, np.pi / 2])
    joint_state = np.radians(np.array([45, -20, 90, -45, 20, -90, 135, 20, -90, -135, -20, 90]))
    q = np.concatenate((base_state, joint_state))

    #r_fl, r_fr, r_rl, r_rr = robot.forward_2D_kinematics(q)
    #r_fl, r_fr, r_rl, r_rr = robot.forward_kinematics(q)
    r_fl, r_fr, r_rl, r_rr = robot.get_positions_motion_to_foot(q)

    print('Generalized coordinates: ', q)
    print('Front left footpoint: ', r_fl)
    print('Front right footpoint: ', r_fr)
    print('Rear left footpoint: ', r_rl)
    print('Rear right footpoint: ', r_rr)

def test_height_profile():

    robot = Astro()

    timesteps = 18

    rz_fl, rz_fr, rz_rl, rz_rr = robot.height_profile(timesteps)

    print('rz_fl: ', rz_fl)
    print('rz_fr: ', rz_fr)
    print('rz_rl: ', rz_rl)
    print('rz_rr: ', rz_rr)

def test_contact_schedule():

    robot = Astro()

    timesteps = 18

    c_fl, c_fr, c_rl, c_rr = robot.contact_flags(timesteps)

    print('c_fl: ', c_fl)
    print('c_fr: ', c_fr)
    print('c_rl: ', c_rl)
    print('c_rr: ', c_rr)


def test_get_body_com():

    robot = Astro()

    base_state = np.array([0, 0, 0.187, 0, 0, 0])
    joint_state = np.radians(np.array([45, -20, 90, -45, 20, -90, 135, 20, -90, -135, -20, 90]))
    q = np.concatenate((base_state, joint_state))

    r_fl = robot.get_position_base_to_body_in_B(q, 'fl', 'foot')
    r_fr = robot.get_position_base_to_body_in_B(q, 'fr', 'leg')
    r_rl = robot.get_position_base_to_body_in_B(q, 'rl', 'thigh')
    r_rr = robot.get_position_base_to_body_in_B(q, 'rr', 'hip')

    print('Generalized coordinates: ', q)
    print('Front left footpoint: ', r_fl)
    print('Front right footpoint: ', r_fr)
    print('Rear left footpoint: ', r_rl)
    print('Rear right footpoint: ', r_rr)

def test_com():

    robot = Astro()

    base_state = np.array([1, 3, 0.187, 0, 0, 0])
    joint_state = np.radians(np.array([10, -30, 100, -45, 0, -30, 45, 10, -10, -15, -70, 20]))
    q = np.concatenate((base_state, joint_state))


    com = robot.center_of_mass_in_M(q)

    cop = robot.center_of_pressure(com, np.array([50, 50, 10]))

    print('Center of Mass (COM): ', com)
    print('Center of Pressure (COP): ', cop)