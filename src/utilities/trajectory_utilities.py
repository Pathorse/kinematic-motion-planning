import numpy as np

# ----------------------------------------------------
# Returns the linear interpolation between two points
# ----------------------------------------------------
def linear_interpolation(start, goal, time_steps):

    # Get num rows and cols
    shape = np.shape(start)

    # Initialize interpolated trajectory
    x = np.zeros((time_steps, shape[0]))

    # Set inital value as the starting point
    x[0] = start

    # Travel step
    dx = (goal - start) / time_steps

    # Interpolate
    for t in range(0, time_steps):

        x[t] = dx * (t + 1)

    return x

def footpoint_xy_interpolation(robot, q_start, desired_net_distance, time_steps):

    # Initialize interpolated steps
    rxy_fl = np.zeros((time_steps, 2))
    rxy_fr = np.zeros((time_steps, 2))
    rxy_rl = np.zeros((time_steps, 2))
    rxy_rr = np.zeros((time_steps, 2))

    period = time_steps // 4

    # Get initial foot positions
    r0_fl, r0_fr, r0_rl, r0_rr = robot.forward_2D_kinematics(q_start)

    dx_fl = (r0_fl + desired_net_distance[:2] - r0_fl) / (period - 1)
    dx_fr = (r0_fr + desired_net_distance[:2] - r0_fr) / (period - 1)
    dx_rl = (r0_rl + desired_net_distance[:2] - r0_rl) / (period - 1)
    dx_rr = (r0_rr + desired_net_distance[:2] - r0_rr) / (period - 1)

    for t in range(time_steps):

        if (t < period):
            rxy_fl[t] = dx_fl * (t) + r0_fl
            rxy_fr[t] = 0
            rxy_rl[t] = 0
            rxy_rr[t] = 0

        elif (t < 2 * period):
            rxy_fl[t] = 0
            rxy_fr[t] = dx_fr * (t - period) + r0_fr
            rxy_rl[t] = 0
            rxy_rr[t] = 0
        
        elif (t < 3 * period):
            rxy_fl[t] = 0
            rxy_fr[t] = 0
            rxy_rl[t] = dx_rl * (t - 2 * period) + r0_rl
            rxy_rr[t] = 0

        else:
            rxy_fl[t] = 0
            rxy_fr[t] = 0
            rxy_rl[t] = 0
            rxy_rr[t] = dx_rr * (t - 3 * period) + r0_rr


    return rxy_fl, rxy_fr, rxy_rl, rxy_rr
    
