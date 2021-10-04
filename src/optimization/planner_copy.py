import numpy as np

from timeit import default_timer

from dynamics.astro import Astro

from pydrake.all import (MathematicalProgram, SnoptSolver, IpoptSolver, eq, ge)

# ----------------------------------------------------
# Add decision variables to the optimization problem
# ----------------------------------------------------
def add_decision_variables(prog, n_q, time_steps):

    # Optimization variables
    q = prog.NewContinuousVariables(rows=time_steps, cols=n_q, name='q')
    x = prog.NewContinuousVariables(rows=time_steps, cols=3, name='x')
    r_fl = prog.NewContinuousVariables(rows=time_steps, cols=3, name='r_fl')
    r_fr = prog.NewContinuousVariables(rows=time_steps, cols=3, name='r_fr')
    r_rl = prog.NewContinuousVariables(rows=time_steps, cols=3, name='r_rl')
    r_rr = prog.NewContinuousVariables(rows=time_steps, cols=3, name='r_rr')
    w_fl = prog.NewContinuousVariables(rows=time_steps, cols=1, name='w_fl')
    w_fr = prog.NewContinuousVariables(rows=time_steps, cols=1, name='w_fr')
    w_rl = prog.NewContinuousVariables(rows=time_steps, cols=1, name='w_rl')
    w_rr = prog.NewContinuousVariables(rows=time_steps, cols=1, name='w_rr')
    
    return q, x, r_fl, r_fr, r_rl, r_rr, w_fl, w_fr, w_rl, w_rr


# ----------------------------------------------------
# Set initial and final condition for the
# optimization problem
# ----------------------------------------------------
def set_initial_and_terminal_position(prog, robot, q_start, q_goal, decision_variables):

    # Unpack state and input
    q, x, r_fl, r_fr, r_rl, r_rr = decision_variables[:6]

    # Initial states
    x_0 = q_start[0:3]
    r_fl_0, r_fr_0, r_rl_0, r_rr_0 = robot.forward_kinematics(q_start)

    # Enforce initial states
    prog.AddLinearConstraint(eq(q[0], q_start))
    prog.AddLinearConstraint(eq(x[0], x_0))
    prog.AddLinearConstraint(eq(r_fl[0], r_fl_0))
    prog.AddLinearConstraint(eq(r_fr[0], r_fr_0))
    prog.AddLinearConstraint(eq(r_rl[0], r_rl_0))
    prog.AddLinearConstraint(eq(r_rr[0], r_rr_0))

    # Enforce final position
    #prog.AddLinearConstraint(eq(q[-1,6:]))

    # Enforce zero final velocity and yaw rate
    #prog.AddLinearConstraint(eq(x[-1,3:], np.array([0, 0, 0])))


# ----------------------------------------------------
# Set state constraints for the
# optimization problem
# ----------------------------------------------------
def set_state_constraints(prog, robot, decision_variables, time_steps):

    # Unpack generalized coordinates, COM position and end
    # effector positions
    q, x, r_fl, r_fr, r_rl, r_rr, w_fl, w_fr, w_rl, w_rr = decision_variables

    # Generalized coordinates bounds
    q_lb = np.array([- np.pi] * robot.n_q)
    q_ub = np.array([np.pi] * robot.n_q)

    # Set zero roll
    #q_lb[3] = 0
    #q_ub[3] = 0

    # Set zero pitch
    #q_lb[4] = 0
    #q_ub[4] = 0

    # COM bounds
    x_lb = np.array([- np.Inf] * 3)
    x_ub = np.array([np.Inf] * 3)

    # Footposition bounds
    r_lb = np.array([- np.Inf, - np.Inf, 0])
    r_ub = np.array([np.Inf, np.Inf, 0])

    # Weight bounds
    w_lb = np.array([0.1])
    w_ub = np.array([1])

    # Add constraints
    for t in range(1, time_steps):
        prog.AddLinearConstraint(q[t], lb=q_lb, ub=q_ub)
        prog.AddLinearConstraint(x[t], lb=x_lb, ub=x_ub)

        prog.AddLinearConstraint(r_fl[t], lb=r_lb, ub=r_ub)
        prog.AddLinearConstraint(r_fr[t], lb=r_lb, ub=r_ub)
        prog.AddLinearConstraint(r_rl[t], lb=r_lb, ub=r_ub)
        prog.AddLinearConstraint(r_rr[t], lb=r_lb, ub=r_ub)

        prog.AddLinearConstraint(w_fl[t], lb=w_lb, ub=w_ub)
        prog.AddLinearConstraint(w_fr[t], lb=w_lb, ub=w_ub)
        prog.AddLinearConstraint(w_rl[t], lb=w_lb, ub=w_ub)
        prog.AddLinearConstraint(w_rr[t], lb=w_lb, ub=w_ub)

# ----------------------------------------------------
# Set the robots forward kinematics as a constraint
# for the optimization problem
# ----------------------------------------------------
def set_kinematics(prog, robot, decision_variables, time_steps):

    # Unpack generalized coordinates, COM position and end
    # effector positions
    q, x, r_fl, r_fr, r_rl, r_rr = decision_variables[:6]

    # Enforce kinematic constraints
    for t in range(1, time_steps):
        residuals = robot.kinematic_residuals(q[t], x[t], r_fl[t], r_fr[t], r_rl[t], r_rr[t])
        for residual in residuals:
            prog.AddConstraint(residual == 0)


# ----------------------------------------------------
# Set the support polygon constraint to ensure that
# the center of pressure lies within
# ----------------------------------------------------
def set_support_polygon(prog, robot, decision_variables, contact_flags, w_lb, time_interval, time_steps):

    # Unpack generalized coordinates, COM position and end
    # effector positions
    q, x, r_fl, r_fr, r_rl, r_rr, w_fl, w_fr, w_rl, w_rr = decision_variables

    # Unpack contact flags
    c_fl, c_fr, c_rl, c_rr = contact_flags

    # Enforce support polygon constraint
    for t in range(2, time_steps - 1):

        # Calculate the COM acceleration using finite
        # differences
        if (t == 0):
            x_ddot = (- 2 * x[t] + x[t+1]) / (time_interval**2)
        elif (t == time_steps - 1):
            x_ddot = (x[t-1] - 2 * x[t]) / (time_interval**2)
        else:
            x_ddot = (x[t-1] - 2 * x[t] + x[t+1]) / (time_interval**2)

        center_of_pressure = robot.center_of_pressure(x[t], x_ddot)

        support_polygon = c_fl[t] * w_fl[t] * r_fl[t] + c_fr[t] * w_fr[t] * r_fr[t] + \
                          c_rl[t] * w_rl[t] * r_rl[t] + c_rr[t] * w_rr[t] * r_rr[t]
        
        residuals = support_polygon - center_of_pressure

        for residual in residuals:
            prog.AddConstraint(residual == 0)

    # Enforce only convex combinations of grounded end-effector
    # positions to define the COP location
    for t in range(1, time_steps):

        residual = c_fl[t] * w_fl[t,0] + c_fr[t] * w_fr[t,0] + \
                   c_rl[t] * w_rl[t,0] + c_rr[t] * w_rr[t,0] - 1

        prog.AddConstraint(residual == 0)


# ----------------------------------------------------
# Set the robots forward kinematics as a constraint
# for the optimization problem
# ----------------------------------------------------
def set_footpoint_constraints(prog, decision_variables, contact_flags, time_steps):

    # Unpack generalized coordinates, COM position and end
    # effector positions
    q, x, r_fl, r_fr, r_rl, r_rr = decision_variables[:6]

    # Unpack contact flags
    c_fl, c_fr, c_rl, c_rr = contact_flags

    # Enfore footpoint contact constraint to avoid
    # foothold slipping
    for t in range(2, time_steps):
        
        if (c_fl[t] != 0):
            #residuals_fl = (r_fl[t-1] - r_fl[t]) * c_fl[t] 
            #for residual in residuals_fl:
            #    #prog.AddConstraint(residual == 0)
            prog.AddLinearConstraint(eq(r_fl[t-1], r_fl[t]))

        if (c_fr[t] != 0):
            #residuals_fr = (r_fr[t-1] - r_fr[t]) * c_fr[t] 
            #for residual in residuals_fr:
            #    prog.AddConstraint(residual == 0)
            prog.AddLinearConstraint(eq(r_fr[t-1], r_fr[t]))

        if (c_rl[t] != 0):
            #residuals_rl = (r_rl[t-1] - r_rl[t]) * c_rl[t] 
            #for residual in residuals_rl:
            #    #prog.AddConstraint(residual == 0)
            prog.AddLinearConstraint(eq(r_rl[t-1], r_rl[t]))

        if (c_rr[t] != 0):
            #residuals_rr = (r_rr[t-1] - r_rr[t]) * c_rr[t] 
            #for residual in residuals_rr:
            #    #prog.AddConstraint(residual == 0)
            prog.AddLinearConstraint(eq(r_rr[t-1], r_rr[t]))


# ----------------------------------------------------
# Set a periodic motion that relates the joint angles
# at the start of the motion and end
# ----------------------------------------------------
def set_periodic_motion(prog, decision_variables, time_steps):

    # Unpack generalized coordinates
    #q = decision_variables[:1]
    q, x = decision_variables[:2]

    # Enforce periodic motion on joint angles
    residuals = q[0, 6:] - q[-1, 6:]

    for residual in residuals:
        prog.AddConstraint(residual == 0)


# ----------------------------------------------------
# Set angular velocity bounds
# ----------------------------------------------------
def set_angular_velocity_bounds(prog, decision_variables, omega_lb, omega_ub, time_interval, time_steps):

    # Unpack generalized coordinates
    #q = decision_variables[:1]
    q, x = decision_variables[:2]

    # Adjust bounds
    omega_lb = np.array([omega_lb] * 12)
    omega_ub = np.array([omega_ub] * 12)

    # Enforce angular velocity bounds
    for t in range(1, time_steps - 1):

        angular_rate = (q[t + 1, 6:] - q[t, 6:]) / time_interval
        
        prog.AddLinearConstraint(angular_rate, lb=omega_lb, ub=omega_ub)


# ----------------------------------------------------
# Set walking and turning speed
# ----------------------------------------------------
def set_walking_and_turning_speed(prog, decision_variables, desired_net_distance, desired_turning_angle, time_steps):

    # Unpack generalized coordinates and COM position
    q, x = decision_variables[:2]

    # Enforce net distance traveled
    net_distance_residuals = x[-1] - x[0] - desired_net_distance

    for residual in net_distance_residuals:
        prog.AddConstraint(residual == 0)

    # Enforce turning angle
    turning_angle_residual = q[-2, 5] - q[1, 5] - desired_turning_angle

    prog.AddConstraint(turning_angle_residual == 0)


# ----------------------------------------------------
# Add the robots forward kinematics as a large
# cost for the optimization problem
# ----------------------------------------------------
def add_kinematics_cost(prog, robot, decision_variables, time_steps):

    # Unpack generalized coordinates, COM position and end
    # effector positions
    q, x, r_fl, r_fr, r_rl, r_rr = decision_variables[:6]

    # Enforce kinematic constraints
    for t in range(1, time_steps - 1):

        # Construct state variables at timestep t
        z = np.concatenate((q[t], x[t], r_fl[t], r_fr[t], r_rl[t], r_rr[t]), axis=None)

        # Add cost
        cost = prog.AddCost(robot.kinematic_cost_function, vars=z)

        cost.evaluator().set_description(f'Kinematic cost iteration: {t}')


# ----------------------------------------------------
# Add generalized coordinate smoothness cost 
# to the optimization problem
# ----------------------------------------------------
def add_smoothness_cost(prog, decision_variables, time_steps):

    # Unpack generalized coordinates, COM position and end
    # effector positions
    q, x = decision_variables[:2]

    # Add cost
    for t in range(2, time_steps - 1):
        
        # Generalized coordinates smoothness
        #if (t == 0):
        #    prog.AddCost((- 2 * q[t] + q[t + 1]).dot(- 2 * q[t] + q[t + 1]))
        #elif (t == time_steps - 1):
        #    prog.AddCost((q[t - 1] - 2 * q[t]).dot(q[t - 1] - 2 * q[t]))
        #else:
        #    prog.AddCost((q[t - 1] - 2 * q[t] + q[t + 1]).dot(q[t - 1] - 2 * q[t] + q[t + 1]))

        prog.AddCost((q[t - 1] - 2 * q[t] + q[t + 1]).dot(q[t - 1] - 2 * q[t] + q[t + 1]))



# ----------------------------------------------------
# Add generalized coordinate smoothness cost 
# to the optimization problem
# ----------------------------------------------------
def add_guide_cost(prog, decision_variables, x_des, r_fl_des, r_fr_des, r_rl_des, r_rr_des, time_steps):

    # Unpack generalized coordinates, COM position and end
    # effector positions
    q, x, r_fl, r_fr, r_rl, r_rr = decision_variables[:6]

    # Add cost
    for t in range(2, time_steps):

        # COM motion style guide
        prog.AddQuadraticCost(x[t] - x_des[t])

        # End-effector motion style guide
        prog.AddQuadraticCost(r_fl[t] - r_fl_des[t])
        prog.AddQuadraticCost(r_fr[t] - r_fr_des[t])
        prog.AddQuadraticCost(r_rl[t] - r_rl_des[t])
        prog.AddQuadraticCost(r_rr[t] - r_rr_des[t])


# ----------------------------------------------------
# Setup Simulation Environment
# ----------------------------------------------------
def run_NLP(robot, q_start, weight_lb, omega_lb, omega_ub, desired_net_distance, desired_turning_angle, \
            x_des, r_fl_des, r_fr_des, r_rl_des, r_rr_des, contact_flags, time_interval, time_steps):

    # Start timer
    NLP_start_time = default_timer()

    # Initialize optimization program
    prog = MathematicalProgram()

    # Optimization variables
    decision_variables = add_decision_variables(prog, robot.n_q, time_steps)

    # Initial and terminal constraints
    set_initial_and_terminal_position(prog, robot, q_start, q_start, decision_variables)

    # State constraints
    set_state_constraints(prog, robot, decision_variables, time_steps)

    # Forward kinematic constraint
    #set_kinematics(prog, robot, decision_variables, time_steps)

    # COP within support polygon constraint
    #set_support_polygon(prog, robot, decision_variables, contact_flags, weight_lb, time_interval, time_steps)

    # Footpoint slipping constraint
    #set_footpoint_constraints(prog, decision_variables, contact_flags, time_steps)

    # Periodic motion constraint
    #set_periodic_motion(prog, decision_variables, time_steps)

    # Angular velocity constraint
    #set_angular_velocity_bounds(prog, decision_variables, omega_lb, omega_ub, time_interval, time_steps)

    # Walking and turning speed constraint
    #set_walking_and_turning_speed(prog, decision_variables, desired_net_distance, desired_turning_angle, time_steps)

    # Cost
    add_kinematics_cost(prog, robot, decision_variables, time_steps)
    add_smoothness_cost(prog, decision_variables, time_steps)
    #add_guide_cost(prog,x_des, r_fl_des, r_fr_des, r_rl_des, r_rr_des, time_steps)

    # Set solver
    #solver = SnoptSolver()
    solver = IpoptSolver()

    # Set verbosity level
    #prog.SetSolverOption(solver.solver_id(), 'print file', 'SNOPT_LOG.txt')
    prog.SetSolverOption(solver.solver_id(), 'print_level', 5)


    # Solve program
    opt_start_time = default_timer()

    result = solver.Solve(prog)

    opt_stop_time = default_timer()

    # Assert the solution
    if result.is_success():

        print("Optimization runtime: ", opt_stop_time - opt_start_time)

        q_opt, x_opt, r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, \
        w_fl_opt, w_fr_opt, w_rl_opt, w_rr_opt = [result.GetSolution(v) for v in decision_variables]

    else:

        print("No feasible solution found.")

        q_opt, x_opt, r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, \
        w_fl_opt, w_fr_opt, w_rl_opt, w_rr_opt = np.zeros((time_steps, robot.n_q)), np.zeros((time_steps, 3)), \
                                                 np.zeros((time_steps, 3)), np.zeros((time_steps, 3)), \
                                                 np.zeros((time_steps, 3)), np.zeros((time_steps, 3)), \
                                                 np.zeros((time_steps, 3)), np.zeros((time_steps, 3)), \
                                                 np.zeros((time_steps, 3)), np.zeros((time_steps, 3))
                                                
    # Stop timer
    NLP_stop_time = default_timer()

    # Print total runtime
    print("NLP Runtime: ", NLP_stop_time - NLP_start_time)

    return q_opt, x_opt, r_fl_opt, r_fr_opt, r_rl_opt, r_rr_opt, w_fl_opt, w_fr_opt, w_rl_opt, w_rr_opt