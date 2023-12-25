import matplotlib.pyplot as plt
import numpy as np
import importlib

from pydrake.all import (
    DiagramBuilder, Simulator, FindResourceOrThrow, MultibodyPlant, PiecewisePolynomial, SceneGraph,
    Parser, JointActuatorIndex, MathematicalProgram, Solve
)

import kinematic_constraints
import dynamics_constraints
importlib.reload(kinematic_constraints)
importlib.reload(dynamics_constraints)
from kinematic_constraints import (
  AddFinalLandingPositionConstraint
)
from dynamics_constraints import (
  AddCollocationConstraints,
  EvaluateDynamics
)

def find_throwing_trajectory(N, initial_state, final_configuration, distance, tf):
  '''
  Parameters:
    N - number of knot points
    initial_state - starting configuration
    distance - target distance to throw the ball

  '''

  builder = DiagramBuilder()
  plant = builder.AddSystem(MultibodyPlant(0.0))
  file_name = "planar_arm.urdf"
  Parser(plant=plant).AddModels(file_name)
  plant.Finalize()
  planar_arm = plant.ToAutoDiffXd()

  plant_context = plant.CreateDefaultContext()
  context = planar_arm.CreateDefaultContext()

  # Dimensions specific to the planar_arm
  n_q = planar_arm.num_positions()
  n_v = planar_arm.num_velocities()
  n_x = n_q + n_v
  n_u = planar_arm.num_actuators()

  # Store the actuator limits here
  effort_limits = np.zeros(n_u)
  for act_idx in range(n_u):
    effort_limits[act_idx] = \
      planar_arm.get_joint_actuator(JointActuatorIndex(act_idx)).effort_limit()
  joint_limits = np.pi * np.ones(n_q)
  vel_limits = 15 * np.ones(n_v)

  # Create the mathematical program
  prog = MathematicalProgram()
  x = np.zeros((N, n_x), dtype="object")
  u = np.zeros((N, n_u), dtype="object")
  for i in range(N):
    x[i] = prog.NewContinuousVariables(n_x, "x_" + str(i))
    u[i] = prog.NewContinuousVariables(n_u, "u_" + str(i))

  t_land = prog.NewContinuousVariables(1, "t_land")

  t0 = 0.0
  timesteps = np.linspace(t0, tf, N)
  x0 = x[0]
  xf = x[-1]

  # DO NOT MODIFY THE LINES ABOVE

  # Add the kinematic constraints (initial state, final state)
  # TODO: Add constraints on the initial state
  prog.AddLinearEqualityConstraint(x0,initial_state)
  prog.AddLinearEqualityConstraint(xf[:2], final_configuration)
  # Add the kinematic constraint on the final state
  AddFinalLandingPositionConstraint(prog, xf, distance, t_land)

  # Add the collocation aka dynamics constraints
  AddCollocationConstraints(prog, planar_arm, context, N, x, u, timesteps)

  # TODO: Add the cost function here
  for i in range(N-1):
      q_cost = 0.5*(timesteps[i+1]-timesteps[i])*((u[i].T @ u[i])+(u[i+1].T @ u[i+1]))
      prog.AddQuadraticCost(q_cost)
  # TODO: Add bounding box constraints on the inputs and qdot 
  for i in range (N):
    prog.AddBoundingBoxConstraint(-effort_limits,effort_limits,u[i])    #torque contraint


    prog.AddBoundingBoxConstraint(-joint_limits,joint_limits,x[i,0:n_q])  #joint position constraint
    prog.AddBoundingBoxConstraint(-vel_limits,vel_limits,x[i,n_q:n_q+n_v]) #joint velocity constraint
    
  # TODO: give the solver an initial guess for x and u using prog.SetInitialGuess(var, value)
  x_g=np.ones((N,n_x))
  for i in range(N):
    x_g[i,:] = np.linspace(0, np.pi , num = n_x)
  u_g= np.zeros((N,n_u))
  for i in range(N):
    u_g[i,:] = np.random.uniform(low=-effort_limits, high=effort_limits, size=(n_u))
  prog.SetInitialGuess(x,x_g)
  prog.SetInitialGuess(u,u_g)
  

  #DO NOT MODIFY THE LINES BELOW

  # Set up solver
  result = Solve(prog)
  
  x_sol = result.GetSolution(x)
  u_sol = result.GetSolution(u)
  t_land_sol = result.GetSolution(t_land)

  print('optimal cost: ', result.get_optimal_cost())
  print('x_sol: ', x_sol)
  print('u_sol: ', u_sol)
  print('t_land: ', t_land_sol)

  print(result.get_solution_result())

  # Reconstruct the trajectory
  xdot_sol = np.zeros(x_sol.shape)
  for i in range(N):
    xdot_sol[i] = EvaluateDynamics(plant, plant_context, x_sol[i], u_sol[i])
  
  x_traj = PiecewisePolynomial.CubicHermite(timesteps, x_sol.T, xdot_sol.T)
  u_traj = PiecewisePolynomial.ZeroOrderHold(timesteps, u_sol.T)

  return x_traj, u_traj, prog, prog.GetInitialGuess(x), prog.GetInitialGuess(u)
  
if __name__ == '__main__':
  N = 5
  initial_state = np.zeros(4)
  final_configuration = np.array([np.pi, 0])
  tf = 3.0
  distance = 15.0
  x_traj, u_traj, prog, _, _ = find_throwing_trajectory(N, initial_state, final_configuration, distance, tf)