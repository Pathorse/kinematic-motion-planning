import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from pydrake.all import *

from dynamics.astro import Astro
#from optimization.planner import run_NLP
from optimization.planner2 import run_NLP
from utilities.plot_utilities import *

from tests.dynamics_test import *
from tests.trajectory_utils_test import *

def main():

    # ----------------------------------------------------
    # Setup Simulation Environment
    # ----------------------------------------------------

    # Robot
    robot = Astro()

    # Set testcase
    testcase = 1

    # Testcase 1
    if (testcase == 1):

        # Numeric parameters
        time_interval = 0.3
        time_steps    = 18

        # Bounds
        weight_lb = 1
        omega_lb = -15
        omega_ub = 15

        # Guidance
        desired_net_distance = np.array([0.4, 0.4, 0])
        desired_turning_angle = 0

        # Desired trajectories
        x_des = 0
        r_fl_des = 0
        r_fr_des = 0
        r_rl_des = 0
        r_rr_des = 0

        # Contact schedule
        contact_flags = robot.contact_flags(time_steps)

        # Initial state
        base_state = np.array([0, 0, 0.2, 0, 0, 0])
        joint_state = np.radians(np.array([45, -20, 90, -45, 20, -90, 135, 20, -90, -135, -20, 90]))
        q_start = np.concatenate((base_state, joint_state))

    # ----------------------------------------------------
    # Optimization
    # ----------------------------------------------------

    # Calculate
    q_opt, x_opt, r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, w_fl_opt, w_fr_opt, w_rl_opt, w_rr_opt = run_NLP(
        robot,
        q_start,
        weight_lb,
        omega_lb,
        omega_ub,
        desired_net_distance,
        desired_turning_angle,
        x_des,
        r_fl_des,
        r_fr_des,
        r_rl_des,
        r_rr_des,
        contact_flags,
        time_interval,
        time_steps
    )


    # ----------------------------------------------------
    # Plotting
    # ----------------------------------------------------
   
    # Timestamps
    timestamps = np.arange(0, time_steps, 1)

    # Plot COM trajectory
    #plot_3D_trajectory(x_opt, title='Center of Mass Trajectory', xlabel='$X$-axis', ylabel='$Y$-axis', zlabel='$Z$-axis', label='$x_{opt}$', color='red')

    # Plot COM 2D trajectory
    #plot_2D_position_trajectory(x_opt, timestamps, title='Centre of Mass Position', color='red')

    # Plot generalized coordinates
    #plot_base_state(q_opt, timestamps, color='black')
    plot_joint_state(q_opt, timestamps, color='black')

    # Plot footpoint trajectories
    plot_footpoint_3D_trajectories(r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, title='Footpoint trajectories', xlabel='x', ylabel='y', zlabel='z', color='blue')
    #plot_footpoint_2D_trajectories(r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, timestamps, title='Footpoint positions', color='blue')
    plot_footpoint_time_trajectories(r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, timestamps, title='Footpoint positions', color='blue')

    # Plot weights
    #plot_weights(w_fl_opt ,w_fr_opt, w_rl_opt, w_rr_opt, timestamps, color='green')

    # Plot XY trajectories for footpoints and COM
    plot_2D_XY_trajectories(x_opt, r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, timestamps)

    # Plot 3D trajectories
    plot_3D_trajectories(x_opt, r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, title='Footpoint trajectories', xlabel='x', ylabel='y', zlabel='z', color='blue')

    plt.show()




    #fig = plt.figure(figsize=(8,4), dpi=100)
    #ax = plt.gca()

  
    # Plot start and goal
    #plt.plot(start[0], start[1], ".b", markersize=10, label='Start')
    #plt.plot(goal[0], goal[1], "*r", markersize=10, label ='Goal')

    ## Plot obstacles
    #for polygon in polygon_obstacles:
    #    polygon.plot()

    ## Plot initial guess
    #plt.plot(x_guess[:,0], x_guess[:,1], '-.', label='A*-dubins trj')

    # Plot optimal trajectory
    #plt.plot(x_opt[:,0], x_opt[:,1], label='Optimal trj')
    #plt.plot(r_fl_opt[:, 1], r_fl_opt[:,2], label='r_fl')

    #plt.plot(timestamps, x_opt[:,0], label='x_opt')
    #plt.plot(timestamps, x_opt[:,1], label='y_opt')
    #plt.plot(timestamps, x_opt[:,2], label='z_opt')
    # Plot usv
    #plot_usv_contour(ax, x_opt, width=10, height=5, tip_height=16)

    # 3D Plot position
    #ax = fig.add_subplot(111, projection='3d')

    #ax.plot(x_opt[:,0], x_opt[:,1], x_opt[:,2], label='x_opt')

    ##plt.axis('equal')
    #plt.xlim(lb[0], ub[0])
    #plt.ylim(lb[1], ub[1])

    #plt.xlabel('Time [s]')
    #plt.ylabel('Position [m]')
    #plt.legend(ncol=2, loc='upper right', borderaxespad=0., fontsize='small')

    #plt.show()

# --------------------------------------------------------------
if __name__ == "__main__":
    main()

    #test_forward_kinematics()
    #test_height_profile()
    #test_contact_schedule()
    #test_linear_interpolation()
    #test_get_body_com()
    #test_com()
    #test_foot_interpolation()