import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D


def plot_2D_XY_trajectories(com_trajectory, fl_trajectory, fr_trajectory, rl_trajectory, rr_trajectory, timestamps, *args, **kwargs):

    # Create figure
    fig, ax = plt.subplots(figsize=(16,9), dpi=100)

    # Plot
    ax.plot(com_trajectory[0,0], com_trajectory[0,1], marker="d", color='black', markersize=10)
    ax.plot(com_trajectory[-1,0], com_trajectory[-1,1], marker="*", color='black', markersize=12)
    ax.plot(com_trajectory[:,0], com_trajectory[:,1], label='COM', color='black', linewidth=3)

    ax.plot(fl_trajectory[0,0], fl_trajectory[0,1], marker="d", color='red', markersize=10)
    ax.plot(fl_trajectory[-1,0], fl_trajectory[-1,1], marker="*", color='red', markersize=12)
    ax.scatter(fl_trajectory[1:-2,0], fl_trajectory[1:-2,1], label='fl', color='red')

    ax.plot(fr_trajectory[0,0], fr_trajectory[0,1], marker="d", color='blue', markersize=10)
    ax.plot(fr_trajectory[-1,0], fr_trajectory[-1,1], marker="*", color='blue', markersize=12)
    ax.scatter(fr_trajectory[1:-2,0], fr_trajectory[1:-2,1], label='fr', color='blue')

    ax.plot(rl_trajectory[0,0], rl_trajectory[0,1], marker="d", color='green', markersize=10)
    ax.plot(rl_trajectory[-1,0], rl_trajectory[-1,1], marker="*", color='green', markersize=12)
    ax.scatter(rl_trajectory[1:-2,0], rl_trajectory[1:-2,1], label='rl', color='green')

    ax.plot(rr_trajectory[0,0], rr_trajectory[0,1], marker="d", color='orange', markersize=10)
    ax.plot(rr_trajectory[-1,0], rr_trajectory[-1,1], marker="*", color='orange', markersize=12)
    ax.scatter(rr_trajectory[1:-2,0], rr_trajectory[1:-2,1], label='rr', color='orange')

    # Settings
    ax.legend(prop={'size': 16})

    fig.suptitle('2D Center of Mass and Footpoint Trajectories', fontsize=24)
    ax.set_xlabel('X [m]', fontsize=20)
    ax.set_ylabel('Y [m]', fontsize=20)


def plot_3D_trajectories(com_trajectory, fl_trajectory, fr_trajectory, rl_trajectory, rr_trajectory, title, xlabel, ylabel, zlabel, *args, **kwargs):

    # Create figure
    fig = plt.figure(figsize=(16,9), dpi=100)

    # Create 3D axes
    ax = plt.axes(projection='3d')

    # Plot
    ax.scatter(com_trajectory[0,0], com_trajectory[0,1], com_trajectory[0,2], color='black', marker="d", s=60)
    ax.scatter(com_trajectory[-1,0], com_trajectory[-1,1], com_trajectory[-1,2], color='black', marker="*", s=80)
    ax.plot(com_trajectory[:,0], com_trajectory[:,1], com_trajectory[:,2], color='black', label='COM', linewidth=3)

    ax.scatter(fl_trajectory[0,0], fl_trajectory[0,1], fl_trajectory[0,2], color='red', marker="o", s=60)
    ax.scatter(fl_trajectory[-1,0], fl_trajectory[-1,1], fl_trajectory[-1,2], color='red', marker="*", s=80)
    ax.plot(fl_trajectory[:,0], fl_trajectory[:,1], fl_trajectory[:,2], label='fl', color='red', linewidth=3)
    
    ax.scatter(fr_trajectory[0,0], fr_trajectory[0,1], fr_trajectory[0,2], color='blue', marker="o", s=60)
    ax.scatter(fr_trajectory[-1,0], fr_trajectory[-1,1], fr_trajectory[-1,2], color='blue', marker="*", s=80)
    ax.plot(fr_trajectory[:,0], fr_trajectory[:,1], fr_trajectory[:,2], label='fr', color='blue', linewidth=3)
    
    ax.scatter(rl_trajectory[0,0], rl_trajectory[0,1], rl_trajectory[0,2], color='green', marker="o", s=60)
    ax.scatter(rl_trajectory[-1,0], rl_trajectory[-1,1], rl_trajectory[-1,2], color='green', marker="*", s=80)
    ax.plot(rl_trajectory[:,0], rl_trajectory[:,1], rl_trajectory[:,2], label='rl', color='green', linewidth=3)

    ax.scatter(rr_trajectory[0,0], rr_trajectory[0,1], rr_trajectory[0,2], color='orange', marker="o", s=60)
    ax.scatter(rr_trajectory[-1,0], rr_trajectory[-1,1], rr_trajectory[-1,2], color='orange', marker="*", s=80)
    ax.plot(rr_trajectory[:,0], rr_trajectory[:,1], rr_trajectory[:,2], label='rr', color='orange', linewidth=3)

    X, Y = np.meshgrid(range(-20,20), range(-20,20))

    Z = 0 * X

    #ax.plot_surface(X, Y, Z, color='grey')

    # Settings
    ax.view_init(30, -75)

    # Get rid of colored axes planes
    # First remove fill
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    #ax.zaxis.pane.fill = False

    # Now set color to white (or whatever is "invisible")
    ax.xaxis.pane.set_edgecolor('w')
    ax.yaxis.pane.set_edgecolor('w')
    #ax.zaxis.pane.set_edgecolor('w')

    fig.suptitle('3D Center of Mass and Footpoint Trajectories', fontsize=24)
    ax.set_xlabel('X [m]', fontsize=20)
    ax.set_ylabel('Y [m]', fontsize=20)
    ax.set_zlabel('Z [m]', fontsize=20)

    ax.legend(prop={'size': 16})

    ax.set_xticks([-1, 0, 1])
    ax.set_yticks([-1, 0, 1])
    ax.set_zticks([0, 0.2, 0.4, 0.6])


    ax.axes.set_xlim3d(left=-1, right=1)
    ax.axes.set_ylim3d(bottom=-1, top=1)
    ax.axes.set_zlim3d(bottom=0, top=0.65)

# ----------------------------------------------------
# Plots a 3D trajectory
# ----------------------------------------------------
def plot_3D_trajectory(trajectory, title, xlabel, ylabel, zlabel, *args, **kwargs):

    # Create figure
    fig = plt.figure()

    # Create 3D axes
    ax = plt.axes(projection='3d')

    # Plot
    ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2], *args, **kwargs)

    # Change initial view
    #ax.view_init(60, 35)

    # Settings
    ax.set_title(title)
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_zlabel(zlabel, fontsize=12)

def plot_2D_position_trajectory(trajectory, timestamps, title, *args, **kwargs):

    # Create figure
    fig = plt.figure(figsize=(16, 9), dpi=100)

    # Create axes
    ax_x = fig.add_subplot(131)
    ax_y = fig.add_subplot(132)
    ax_z = fig.add_subplot(133)

    # Plot
    ax_x.plot(timestamps, trajectory[:,0], *args, **kwargs)
    ax_y.plot(timestamps, trajectory[:,1], *args, **kwargs)
    ax_z.plot(timestamps, trajectory[:,2], *args, **kwargs)

    # Settings
    fig.suptitle(title)
    ax_x.set_title('x-position')
    ax_y.set_title('y-position')
    ax_z.set_title('z-position')

    ax_x.set_xlabel('Time [s]')
    ax_x.set_ylabel('Position [m]')
    ax_y.set_xlabel('Time [s]')
    ax_y.set_ylabel('Position [m]')
    ax_z.set_xlabel('Time [s]')
    ax_z.set_ylabel('Position [m]')

def plot_base_state(gen_coord, timestamps, *args, **kwargs):

    # Create figure and axes
    fig, axes = plt.subplots(figsize=(16,9), dpi=100, ncols=3, nrows=2, constrained_layout=True)
    
    # Plot
    axes[0, 0].plot(timestamps, gen_coord[:,0], *args, **kwargs)
    axes[0, 1].plot(timestamps, gen_coord[:,1], *args, **kwargs)
    axes[0, 2].plot(timestamps, gen_coord[:,2], *args, **kwargs)
    axes[1, 0].plot(timestamps, 180 / np.pi * gen_coord[:,3], *args, **kwargs)
    axes[1, 1].plot(timestamps, 180 / np.pi * gen_coord[:,4], *args, **kwargs)
    axes[1, 2].plot(timestamps, 180 / np.pi * gen_coord[:,5], *args, **kwargs)

    # Settings
    fig.suptitle('Base State')
    axes[0, 0].set_title('x-position')
    axes[0, 1].set_title('y-position')
    axes[0, 2].set_title('z-position')
    axes[1, 0].set_title('Roll')
    axes[1, 1].set_title('Pitch')
    axes[1, 2].set_title('Yaw')

    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Position [m]')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Position [m]')
    axes[0, 2].set_xlabel('Time [s]')
    axes[0, 2].set_ylabel('Position [m]')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Angle [deg]')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Angle [deg]')
    axes[1, 2].set_xlabel('Time [s]')
    axes[1, 2].set_ylabel('Angle [deg]')


def plot_joint_state(gen_coord, timestamps, *args, **kwargs):

    # Create figure and axes
    fig, axes = plt.subplots(figsize=(16,9), dpi=100, ncols=3, nrows=4, constrained_layout=True)

    # Plot
    axes[0, 0].plot(timestamps, 180 / np.pi * gen_coord[:,6], *args, **kwargs)
    axes[0, 1].plot(timestamps, 180 / np.pi * gen_coord[:,7], *args, **kwargs)
    axes[0, 2].plot(timestamps, 180 / np.pi * gen_coord[:,8], *args, **kwargs)
    axes[1, 0].plot(timestamps, 180 / np.pi * gen_coord[:,9], *args, **kwargs)
    axes[1, 1].plot(timestamps, 180 / np.pi * gen_coord[:,10], *args, **kwargs)
    axes[1, 2].plot(timestamps, 180 / np.pi * gen_coord[:,11], *args, **kwargs)
    axes[2, 0].plot(timestamps, 180 / np.pi * gen_coord[:,12], *args, **kwargs)
    axes[2, 1].plot(timestamps, 180 / np.pi * gen_coord[:,13], *args, **kwargs)
    axes[2, 2].plot(timestamps, 180 / np.pi * gen_coord[:,14], *args, **kwargs)
    axes[3, 0].plot(timestamps, 180 / np.pi * gen_coord[:,15], *args, **kwargs)
    axes[3, 1].plot(timestamps, 180 / np.pi * gen_coord[:,16], *args, **kwargs)
    axes[3, 2].plot(timestamps, 180 / np.pi * gen_coord[:,17], *args, **kwargs)

    # Settings
    fig.suptitle('Joint States')
    axes[0, 0].set_title('Front left hip yaw')
    axes[0, 1].set_title('Front left hip pitch')
    axes[0, 2].set_title('Front left knee pitch')
    axes[1, 0].set_title('Front right hip yaw')
    axes[1, 1].set_title('Front right hip pitch')
    axes[1, 2].set_title('Front right knee pitch')
    axes[2, 0].set_title('Rear left hip yaw')
    axes[2, 1].set_title('Rear left hip pitch')
    axes[2, 2].set_title('Rear left knee pitch')
    axes[3, 0].set_title('Rear right hip yaw')
    axes[3, 1].set_title('Rear right hip pitch')
    axes[3, 2].set_title('Rear right knee pitch')

    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Angle [deg]')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Angle [deg]')
    axes[0, 2].set_xlabel('Time [s]')
    axes[0, 2].set_ylabel('Angle [deg]')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Angle [deg]')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Angle [deg]')
    axes[1, 2].set_xlabel('Time [s]')
    axes[1, 2].set_ylabel('Angle [deg]')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('Angle [deg]')
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_ylabel('Angle [deg]')
    axes[2, 2].set_xlabel('Time [s]')
    axes[2, 2].set_ylabel('Angle [deg]')
    axes[3, 0].set_xlabel('Time [s]')
    axes[3, 0].set_ylabel('Angle [deg]')
    axes[3, 1].set_xlabel('Time [s]')
    axes[3, 1].set_ylabel('Angle [deg]')
    axes[3, 2].set_xlabel('Time [s]')
    axes[3, 2].set_ylabel('Angle [deg]')



def plot_footpoint_3D_trajectories(fl_trajectory, fr_trajectory, rl_trajectory, rr_trajectory, title, xlabel, ylabel, zlabel, *args, **kwargs):

    # Create figure
    fig = plt.figure(figsize=(16,9), dpi=100)

    # Create 3D axes
    ax_fl = plt.subplot(141, projection='3d')
    ax_fr = plt.subplot(142, projection='3d')
    ax_rl = plt.subplot(143, projection='3d')
    ax_rr = plt.subplot(144, projection='3d')

    # Plot
    ax_fl.plot(fl_trajectory[:,0], fl_trajectory[:,1], fl_trajectory[:,2], *args, **kwargs)
    ax_fr.plot(fr_trajectory[:,0], fr_trajectory[:,1], fr_trajectory[:,2], *args, **kwargs)
    ax_rl.plot(rl_trajectory[:,0], rl_trajectory[:,1], rl_trajectory[:,2], *args, **kwargs)
    ax_rr.plot(rr_trajectory[:,0], rr_trajectory[:,1], rr_trajectory[:,2], *args, **kwargs)

    # Change initial view
    #ax_fl.view_init(45, 35)
    #ax_fr.view_init(45, 35)
    #ax_rl.view_init(45, 35)
    #ax_rr.view_init(45, 35)

    # Settings
    fig.suptitle('Footpoint Trajectories')
    ax_fl.set_title('Front Left')
    ax_fr.set_title('Front Right')
    ax_rl.set_title('Rear Left')
    ax_rr.set_title('Rear Right')
    ax_fl.set_xlabel(xlabel, fontsize=12)
    ax_fl.set_ylabel(ylabel, fontsize=12)
    ax_fl.set_zlabel(zlabel, fontsize=12)
    ax_fr.set_xlabel(xlabel, fontsize=12)
    ax_fr.set_ylabel(ylabel, fontsize=12)
    ax_fr.set_zlabel(zlabel, fontsize=12)
    ax_rl.set_xlabel(xlabel, fontsize=12)
    ax_rl.set_ylabel(ylabel, fontsize=12)
    ax_rl.set_zlabel(zlabel, fontsize=12)
    ax_rr.set_xlabel(xlabel, fontsize=12)
    ax_rr.set_ylabel(ylabel, fontsize=12)
    ax_rr.set_zlabel(zlabel, fontsize=12)


def plot_footpoint_2D_trajectories(fl_trajectory, fr_trajectory, rl_trajectory, rr_trajectory, timestamps, title, *args, **kwargs):

    # Create figure
    fig, ax = plt.subplots(figsize=(16,9), dpi=100)

    # Plot
    ax.scatter(fl_trajectory[:,0], fl_trajectory[:,1], label='fl', color='red')
    ax.scatter(fr_trajectory[:,0], fr_trajectory[:,1], label='fr', color='blue')
    ax.scatter(rl_trajectory[:,0], rl_trajectory[:,1], label='rl', color='green')
    ax.scatter(rr_trajectory[:,0], rr_trajectory[:,1], label='rr', color='orange')

    # Settings
    fig.suptitle('Footpoint Trajectories')


def plot_footpoint_time_trajectories(fl_trajectory, fr_trajectory, rl_trajectory, rr_trajectory, timestamps, title, *args, **kwargs):

    # Create figure and axes
    fig, axes = plt.subplots(figsize=(16,9), dpi=100, ncols=3, nrows=4, constrained_layout=True)

    # Plot
    axes[0, 0].plot(timestamps, fl_trajectory[:,0], *args, **kwargs)
    axes[0, 1].plot(timestamps, fl_trajectory[:,1], *args, **kwargs)
    axes[0, 2].plot(timestamps, fl_trajectory[:,2], *args, **kwargs)
    axes[1, 0].plot(timestamps, fr_trajectory[:,0], *args, **kwargs)
    axes[1, 1].plot(timestamps, fr_trajectory[:,1], *args, **kwargs)
    axes[1, 2].plot(timestamps, fr_trajectory[:,2], *args, **kwargs)
    axes[2, 0].plot(timestamps, rl_trajectory[:,0], *args, **kwargs)
    axes[2, 1].plot(timestamps, rl_trajectory[:,1], *args, **kwargs)
    axes[2, 2].plot(timestamps, rl_trajectory[:,2], *args, **kwargs)
    axes[3, 0].plot(timestamps, rr_trajectory[:,0], *args, **kwargs)
    axes[3, 1].plot(timestamps, rr_trajectory[:,1], *args, **kwargs)
    axes[3, 2].plot(timestamps, rr_trajectory[:,2], *args, **kwargs)

    # Settings
    fig.suptitle(title)
    axes[0, 0].set_title('Front left x-position')
    axes[0, 1].set_title('Front left y-position')
    axes[0, 2].set_title('Front left z-position')
    axes[1, 0].set_title('Front right x-position')
    axes[1, 1].set_title('Front right y-position')
    axes[1, 2].set_title('Front right z-position')
    axes[2, 0].set_title('Rear left x-position')
    axes[2, 1].set_title('Rear left y-position')
    axes[2, 2].set_title('Rear left z-position')
    axes[3, 0].set_title('Rear right x-position')
    axes[3, 1].set_title('Rear right y-position')
    axes[3, 2].set_title('Rear right z-position')

    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Position [m]')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Position [m]')
    axes[0, 2].set_xlabel('Time [s]')
    axes[0, 2].set_ylabel('Position [m]')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Position [m]')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Position [m]')
    axes[1, 2].set_xlabel('Time [s]')
    axes[1, 2].set_ylabel('Position [m]')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('Position [m]')
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_ylabel('Position [m]')
    axes[2, 2].set_xlabel('Time [s]')
    axes[2, 2].set_ylabel('Position [m]')
    axes[3, 0].set_xlabel('Time [s]')
    axes[3, 0].set_ylabel('Position [m]')
    axes[3, 1].set_xlabel('Time [s]')
    axes[3, 1].set_ylabel('Position [m]')
    axes[3, 2].set_xlabel('Time [s]')
    axes[3, 2].set_ylabel('Position [m]')


def plot_weights(w_fl, w_fr, w_rl, w_rr, timestamps, *args, **kwargs):

    # Create figure and axes
    fig, axes = plt.subplots(figsize=(16,9), dpi=100, ncols=1, nrows=4, constrained_layout=True)

    # Plot
    axes[0].plot(timestamps, w_fl, *args, **kwargs)
    axes[1].plot(timestamps, w_fr, *args, **kwargs)
    axes[2].plot(timestamps, w_rl, *args, **kwargs)
    axes[3].plot(timestamps, w_rr, *args, **kwargs)

    # Settings
    fig.suptitle('Weights')
    axes[0].set_title('Front left weight')
    axes[1].set_title('Front right weight')
    axes[2].set_title('Rear left weight')
    axes[3].set_title('Rear right weight')

    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Value')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Value')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Value')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('Value')