import numpy as np

import pydrake.symbolic as sym
import pydrake.math as mu

from utilities.math_utilities import EulerAnglesZYX

from pydrake.all import *
from pydrake.symbolic import Variable

from pydrake.all import (RotationMatrix, RigidTransform)

class Astro(object):

    def __init__(
        self,
        name="ASTRO",
        n_q=18,
        L1=0.130,
        L2=0.220,
        L3=0.279,
        LC1=0.1030,
        LC2=0.0257,
        LC3=0.1069,
        M0=10.1,
        M1=1.179,
        M2=1.394,
        M3=0.283,
        position_base_to_fl_in_B=np.array([0.151, 0.151, 0]),
        position_base_to_fr_in_B=np.array([0.151, -0.151, 0]),
        position_base_to_rl_in_B=np.array([-0.151, 0.151, 0]),
        position_base_to_rr_in_B=np.array([-0.151, -0.151, 0])
    ):

        # Declare name
        self.name = name

        # Declare number of state variables
        self.n_q = n_q

        # Declare link lengths
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.LC1 = LC1
        self.LC2 = LC2
        self.LC3 = LC3

        # Declare masses
        self.M0 = M0
        self.M1 = M1
        self.M2 = M2
        self.M3 = M3

        # Declare position vectors
        self.position_base_to_fl_in_B = position_base_to_fl_in_B
        self.position_base_to_fr_in_B = position_base_to_fr_in_B
        self.position_base_to_rl_in_B = position_base_to_rl_in_B
        self.position_base_to_rr_in_B = position_base_to_rr_in_B

    # ----------------------------------------------------
    # Define kinematic residuals
    # ----------------------------------------------------
    def kinematic_residuals(self, q, x, r_fl, r_fr, r_rl, r_rr):

        fl_fk, fr_fk, rl_fk, rr_fk = self.get_positions_motion_to_foot(q)
        com = self.center_of_mass_in_M(q)

        com_residual = com - x
        fl_residual = fl_fk - r_fl
        fr_residual = fr_fk - r_fr
        rl_residual = rl_fk - r_rl
        rr_residual = rr_fk - r_rr

        residuals = np.concatenate((com_residual, fl_residual, fr_residual, rl_residual, rr_residual))

        return residuals        

    # ----------------------------------------------------
    # Define kinematic 2D residuals
    # ----------------------------------------------------
    def kinematic_2D_residuals(self, q, x, r_fl, r_fr, r_rl, r_rr):

        fl_fk, fr_fk, rl_fk, rr_fk = self.forward_2D_kinematics(q)

        com_residual = np.array([q[0], q[1], q[2]]) - x
        fl_residual = fl_fk - r_fl
        fr_residual = fr_fk - r_fr
        rl_residual = rl_fk - r_rl
        rr_residual = rr_fk - r_rr

        residuals = np.concatenate((com_residual, fl_residual, fr_residual, rl_residual, rr_residual))

        return residuals        

    # ----------------------------------------------------
    # Define kinematic cost function
    # ----------------------------------------------------
    def kinematic_cost_function(self, z):

        q = z[0:18]
        x = z[18:21]
        r_fl = z[21:24]
        r_fr = z[24:27]
        r_rl = z[27:30]
        r_rr = z[30:33]

        #print('q: ', q)
        #print('x: ', x)
        #print('r_fl: ', r_fl)
        #print('r_fr: ', r_fr)
        #print('r_rl: ', r_rl)
        #print('r_rr: ', r_rr)

        fl_fk, fr_fk, rl_fk, rr_fk = self.get_positions_motion_to_foot(q)
        com = self.center_of_mass_in_M(q)

        com_residual = com - x
        fl_residual = fl_fk - r_fl
        fr_residual = fr_fk - r_fr
        rl_residual = rl_fk - r_rl
        rr_residual = rr_fk - r_rr

        residuals = np.concatenate((com_residual, fl_residual, fr_residual, rl_residual, rr_residual))

        #return 10**4 * residuals.dot(residuals)    
        
        return 10**4 * (com_residual.dot(com_residual) + fl_residual.dot(fl_residual) + fr_residual.dot(fr_residual) + \
                        rl_residual.dot(rl_residual) + rr_residual.dot(rr_residual))

    # ----------------------------------------------------
    # Define kinematic 2D cost function
    # ----------------------------------------------------
    def kinematic_2D_cost_function(self, z):

        q = z[0:18]
        x = z[18:21]
        rxy_fl = z[21:23]
        rxy_fr = z[23:25]
        rxy_rl = z[25:27]
        rxy_rr = z[27:29]

        #print('q: ', q)
        #print('x: ', x)
        #print('r_fl: ', r_fl)
        #print('r_fr: ', r_fr)
        #print('r_rl: ', r_rl)
        #print('r_rr: ', r_rr)

        fl_fk, fr_fk, rl_fk, rr_fk = self.forward_2D_kinematics(q)

        com_residual = np.array([q[0], q[1], q[2]]) - x
        fl_residual = fl_fk - rxy_fl
        fr_residual = fr_fk - rxy_fr
        rl_residual = rl_fk - rxy_rl
        rr_residual = rr_fk - rxy_rr
        
        return 10**4 * (com_residual.dot(com_residual) + fl_residual.dot(fl_residual) + fr_residual.dot(fr_residual) + \
                        rl_residual.dot(rl_residual) + rr_residual.dot(rr_residual))

    # ----------------------------------------------------
    # Define forward kinematics
    # ----------------------------------------------------
    def forward_kinematics(self, q, type='np'):

        # Define states
        x = q[0]
        y = q[1]
        z = q[2]
        roll = q[3]
        pitch = q[4]
        yaw = q[5]

        # Rotation matrix from world to body (transform from body to world)
        rotation_W_to_B = EulerAnglesZYX(yaw, pitch, roll, type)

        #print('Rotation_W_to_B: ', rotation_W_to_B)

        # Define positional vector from base to foot
        position_base_to_foot_fl_in_B = self.get_position_base_to_body_in_B(q, 'fl', 'foot', type)
        position_base_to_foot_fr_in_B = self.get_position_base_to_body_in_B(q, 'fr', 'foot', type)
        position_base_to_foot_rl_in_B = self.get_position_base_to_body_in_B(q, 'rl', 'foot', type)
        position_base_to_foot_rr_in_B = self.get_position_base_to_body_in_B(q, 'rr', 'foot', type)

        # Define positional vector from world zero to base
        position_world_to_base_in_W = np.array([x, y, z])

        # Find positional vectors from world zero to end effectors
        position_world_to_foot_fl_in_W = position_world_to_base_in_W + rotation_W_to_B.dot(position_base_to_foot_fl_in_B)
        position_world_to_foot_fr_in_W = position_world_to_base_in_W + rotation_W_to_B.dot(position_base_to_foot_fr_in_B)
        position_world_to_foot_rl_in_W = position_world_to_base_in_W + rotation_W_to_B.dot(position_base_to_foot_rl_in_B)
        position_world_to_foot_rr_in_W = position_world_to_base_in_W + rotation_W_to_B.dot(position_base_to_foot_rr_in_B)

        return position_world_to_foot_fl_in_W, position_world_to_foot_fr_in_W, position_world_to_foot_rl_in_W, position_world_to_foot_rr_in_W

    # ----------------------------------------------------
    # Define forward planar kinematics
    # ----------------------------------------------------
    def forward_2D_kinematics(self, q):

        position_world_to_foot_fl_in_W, position_world_to_foot_fr_in_W, position_world_to_foot_rl_in_W, position_world_to_foot_rr_in_W = self.forward_kinematics(q)

        position_world_to_foot_fl_in_W = np.delete(position_world_to_foot_fl_in_W, 2)
        position_world_to_foot_fr_in_W = np.delete(position_world_to_foot_fr_in_W, 2)
        position_world_to_foot_rl_in_W = np.delete(position_world_to_foot_rl_in_W, 2)
        position_world_to_foot_rr_in_W = np.delete(position_world_to_foot_rr_in_W, 2)

        return position_world_to_foot_fl_in_W, position_world_to_foot_fr_in_W, position_world_to_foot_rl_in_W, position_world_to_foot_rr_in_W


    # ----------------------------------------------------
    # Define position from motion frame origin to foot 
    # positions
    # ----------------------------------------------------
    def get_positions_motion_to_foot(self, q, type='np'):

        # Define states
        x = q[0]
        y = q[1]
        z = q[2]
        roll = q[3]
        pitch = q[4]
        yaw = q[5]

        # Rotation matrix from motion to body (transform from body to motion)
        rotation_M_to_B = EulerAnglesZYX(0, pitch, roll, type)

        # Define positional vector from base to foot
        position_base_to_foot_fl_in_B = self.get_position_base_to_body_in_B(q, 'fl', 'foot', type)
        position_base_to_foot_fr_in_B = self.get_position_base_to_body_in_B(q, 'fr', 'foot', type)
        position_base_to_foot_rl_in_B = self.get_position_base_to_body_in_B(q, 'rl', 'foot', type)
        position_base_to_foot_rr_in_B = self.get_position_base_to_body_in_B(q, 'rr', 'foot', type)

        # Define positional vector from world zero to base
        position_motion_to_base_in_M = np.array([x, y, z])

        # Find positional vectors from world zero to end effectors
        position_motion_to_foot_fl_in_M = position_motion_to_base_in_M + rotation_M_to_B.dot(position_base_to_foot_fl_in_B)
        position_motion_to_foot_fr_in_M = position_motion_to_base_in_M + rotation_M_to_B.dot(position_base_to_foot_fr_in_B)
        position_motion_to_foot_rl_in_M = position_motion_to_base_in_M + rotation_M_to_B.dot(position_base_to_foot_rl_in_B)
        position_motion_to_foot_rr_in_M = position_motion_to_base_in_M + rotation_M_to_B.dot(position_base_to_foot_rr_in_B)

        return position_motion_to_foot_fl_in_M, position_motion_to_foot_fr_in_M, position_motion_to_foot_rl_in_M, position_motion_to_foot_rr_in_M


    # ----------------------------------------------------
    # Define position vector from base to foot in body
    # given a legtype
    # ----------------------------------------------------
    def get_position_base_to_body_in_B(self, q, legtype, bodytype, type='np'):

        m = sym if type == 'sym' else np

        position_base_to_hip_in_B = np.array([0, 0, 0])

        theta_hy = 0
        theta_hp = 0
        theta_kp = 0

        l1 = 0
        l2 = 0
        l3 = 0

        if (bodytype == 'base'):

            print("Position from base to base is zero, check for mistakes..")
        
        elif (bodytype == 'hip'):

            l1 = self.LC1
        
        elif (bodytype == 'thigh'):

            l1 = self.L1
            l2 = self.LC2
        
        elif (bodytype == 'leg'):
            
            l1 = self.L1
            l2 = self.L2
            l3 = self.LC3
        
        elif (bodytype == 'foot'):

            l1 = self.L1
            l2 = self.L2
            l3 = self.L3

        else:

            print("[get_position_base_to_body_in_B] Could not determine bodytype")
            

        if (legtype == 'fl'):

            position_base_to_hip_in_B = self.position_base_to_fl_in_B

            theta_hy = q[6]
            theta_hp = q[7]
            theta_kp = q[8]

        elif (legtype == 'fr'):

            position_base_to_hip_in_B = self.position_base_to_fr_in_B

            theta_hy = q[9]
            theta_hp = q[10]
            theta_kp = q[11]

        elif (legtype == 'rl'):

            position_base_to_hip_in_B = self.position_base_to_rl_in_B

            theta_hy = q[12]
            theta_hp = q[13]
            theta_kp = q[14]
        
        elif (legtype == 'rr'):

            position_base_to_hip_in_B = self.position_base_to_rr_in_B

            theta_hy = q[15]
            theta_hp = q[16]
            theta_kp = q[17]

        else:

            print('[get_position_base_to_body_in_B] Could not determine legtype')


        position_hip_to_body_in_B_x = m.cos(theta_hy) * (l1 + l2 * m.cos(theta_hp) + l3 * m.cos(theta_hp + theta_kp))
        position_hip_to_body_in_B_y = m.sin(theta_hy) * (l1 + l2 * m.cos(theta_hp) + l3 * m.cos(theta_hp + theta_kp))
        position_hip_to_body_in_B_z = - l2 * m.sin(theta_hp) - l3 * m.sin(theta_hp + theta_kp)

        if (legtype == 'fr' or legtype == 'rl'):
            position_hip_to_body_in_B_z = - position_hip_to_body_in_B_z

        position_hip_to_body_in_B = np.array([position_hip_to_body_in_B_x, position_hip_to_body_in_B_y, position_hip_to_body_in_B_z])


        return position_base_to_hip_in_B + position_hip_to_body_in_B


    # ----------------------------------------------------
    # Define center of pressure
    # ----------------------------------------------------
    def center_of_mass_in_M(self, q, type='np'):

        # List of bodies
        bodies = ['hip', 'thigh', 'leg']

        # List of legs
        legs = ['fl', 'fr', 'rl', 'rr']

        # Total mass
        M = self.M0 + 4 * self.M1 + 4 * self.M2 + 4 * self.M3

        # Define com
        com = np.zeros(3, dtype=Variable)

        # Summarize each links center of mass
        for body in bodies:
            for leg in legs:
                if (body == 'hip'):
                
                    com += self.M1 * self.get_position_base_to_body_in_B(q, leg, body, type)
                
                elif (body == 'thigh'):

                    com += self.M2 * self.get_position_base_to_body_in_B(q, leg, body, type)

                elif (body == 'leg'):

                    com += self.M3 * self.get_position_base_to_body_in_B(q, leg, body, type)

        # Calculate center of mass
        com = com / M

        # Define states
        x = q[0]
        y = q[1]
        z = q[2]
        roll = q[3]
        pitch = q[4]
        yaw = q[5]

        # Rotation matrix from motion to body (transform from body to motion)
        rotation_M_to_B = EulerAnglesZYX(0, pitch, roll, type)

        # Define positional vector from world zero to base
        position_motion_to_base_in_M = np.array([x, y, z])

        # Transform COM from base-frame to motion frame
        com = position_motion_to_base_in_M + rotation_M_to_B.dot(com)

        return com

    
    # ----------------------------------------------------
    # Define center of pressure
    # ----------------------------------------------------
    def center_of_pressure(self, x, x_ddot):

        # Define gravity constant
        g = 9.81

        # Extract vertical (z-directional) position and acceleration
        x_z = x[2]
        x_z_ddot = x_ddot[2]

        # Calculate center of pressure
        cop = x - (x_z / (x_z_ddot + g)) * x_ddot

        return cop

    # ----------------------------------------------------
    # Define 2D center of pressure
    # ----------------------------------------------------
    def center_of_pressure_2D(self, x, x_ddot):

        cop = self.center_of_pressure(x, x_ddot)

        cop = np.delete(cop, 2)

        return cop

    # ----------------------------------------------------
    # Define contact flags
    # ----------------------------------------------------
    def contact_flags(self, time_steps):

        period = (time_steps - 2) // 4

        c_fl = np.zeros(time_steps)
        c_fr = np.zeros(time_steps)
        c_rl = np.zeros(time_steps)
        c_rr = np.zeros(time_steps)

        c_fl[0] = 1
        c_fr[0] = 1
        c_rl[0] = 1
        c_rr[0] = 1

        c_fl[-1] = 1
        c_fr[-1] = 1
        c_rl[-1] = 1
        c_rr[-1] = 1

        for t in range(1, time_steps - 1):

            if (t < period + 1):
                c_fl[t] = 0
                c_fr[t] = 1
                c_rl[t] = 1
                c_rr[t] = 1

            elif (t < 2 * period + 1):
                c_fl[t] = 1
                c_fr[t] = 0
                c_rl[t] = 1
                c_rr[t] = 1
            
            elif (t < 3 * period + 1):
                c_fl[t] = 1
                c_fr[t] = 1
                c_rl[t] = 0
                c_rr[t] = 1

            else:
                c_fl[t] = 1
                c_fr[t] = 1
                c_rl[t] = 1
                c_rr[t] = 0

        return c_fl, c_fr, c_rl, c_rr

    # ----------------------------------------------------
    # Define footpoint height profile
    # ----------------------------------------------------
    def height_profile(self, time_steps):

        amplitude = 0.10
        period = (time_steps - 2) // 4

        rz_fl = np.zeros(time_steps)
        rz_fr = np.zeros(time_steps)
        rz_rl = np.zeros(time_steps)
        rz_rr = np.zeros(time_steps)

        rz_fl[0] = 0
        rz_fr[0] = 0
        rz_rl[0] = 0
        rz_rr[0] = 0

        rz_fl[-1] = 0
        rz_fr[-1] = 0
        rz_rl[-1] = 0
        rz_rr[-1] = 0

        #print('Period: ', period)

        for t in range(1, time_steps - 1):

            if (t < period + 1):
                rz_fl[t] = amplitude * np.sin( t * np.pi / period)
                rz_fr[t] = 0
                rz_rl[t] = 0
                rz_rr[t] = 0

            elif (t < 2 * period + 1):
                rz_fl[t] = 0
                rz_fr[t] = amplitude * np.sin( (t - period) * np.pi / period)
                rz_rl[t] = 0
                rz_rr[t] = 0
            
            elif (t < 3 * period + 1):
                rz_fl[t] = 0
                rz_fr[t] = 0
                rz_rl[t] = amplitude * np.sin( (t - 2 * period) * np.pi / period)
                rz_rr[t] = 0

            else:
                rz_fl[t] = 0
                rz_fr[t] = 0
                rz_rl[t] = 0
                rz_rr[t] = amplitude * np.sin( (t - 3 * period) * np.pi / period)

        return rz_fl, rz_fr, rz_rl, rz_rr
    
