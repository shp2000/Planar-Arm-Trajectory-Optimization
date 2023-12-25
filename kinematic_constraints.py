import numpy as np
from pydrake.autodiffutils import AutoDiffXd

def cos(theta):
  return AutoDiffXd.cos(theta)
def sin(theta):
  return AutoDiffXd.sin(theta)

def landing_constraint(vars):
  '''
  Impose a constraint such that if the ball is released at final state xf, 
  it will land a distance d from the base of the robot 
  '''
  l = 1
  g = 9.81
  constraint_eval = np.zeros((3,), dtype=AutoDiffXd)
  q = vars[:2]
  qdot = vars[2:4]
  t_land = vars[-1]
  pos = np.array([[-l*sin(q[0]) - l*sin(q[0] + q[1])],
                  [-l*cos(q[0]) - l*cos(q[0] + q[1])]])
  vel = np.array([[-l*qdot[1]*cos(q[0] + q[1]) + qdot[0]*(-l*cos(q[0]) - l*cos(q[0] + q[1]))], 
                  [l*qdot[1]*sin(q[0] + q[1]) + qdot[0]*(l*sin(q[0]) + l*sin(q[0] + q[1]))]])

  # TODO: Express the landing constraint as a function of q, qdot, and t_land
  # constraint_eval[0]: Eq (23)
  # constraint_eval[1]: Eq (24)
  # constraint_eval[2]: Eq (26)
  constraint_eval[0] = pos[1,0] + t_land * vel[1,0] - 0.5 * g * t_land**2 
  constraint_eval[1] = pos[0,0] + t_land * vel[0,0]
  constraint_eval[2] = pos[1,0]

  return constraint_eval

def AddFinalLandingPositionConstraint(prog, xf, d, t_land):

  # TODO: Add the landing distance equality constraint as a system of inequality constraints 
  # using prog.AddConstraint(landing_constraint, lb, ub, vars) 

  # TODO: Add a constraint that t_land is positive
  #lb=np.array([0,d,2.0])
  #ub = lb
  prog.AddConstraint(landing_constraint,np.array([0,d,2.0]),np.array([0,d,2.0]),np.hstack([xf,t_land[0]]))
  prog.AddConstraint(t_land[0]>=0)
