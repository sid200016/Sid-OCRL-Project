import pybullet as p
import pybullet_data
import time
import numpy as np
from Robot_Init import Robot
import sympy as sy
from sympy.utilities.codegen import codegen
robot_id = r"C:\Users\sidas\OneDrive\Desktop\Research\Robot_sim\robot_arm_3\urdf\robot_arm_3.urdf"
#HomeConfig
joint_positions = [0, 0,0, 0, 0, np.pi/2, 0,0]  # Change based on your desired configuration

bot1  = Robot(joint_positions, robot_id)
dt = 0.02
# bot1

# info = bot1.l1_transform()
# # print("0", info[0])
# # print("1", info[1])
# # print("2", info[2])
# # print("3", info[3])
# # print("4", info[4])
# # print("5", info[5])
# base = bot1.get_base_state()
# # print("b", base)
# angles = bot1.get_joint_states()
# # print("ja", angles)
dt = 0.02
tf = 5
t = np.linspace(0, 5, 1000)
N = len(t)
nx = 14
nu = 7
Q = 1.0*np.eye(nx)
Qf = 10.0*Q
R = 0.5*np.eye(nu)
Mass = bot1.getMassMatrix(joint_positions)
q_init = joint_positions[1:len(joint_positions)]
tau_goal = np.array([0, 2, 2,2,2,2,0]).reshape((7, -1))
q_goal = [0, np.pi,0,0,0,0, 0]
dq_goal = [0,0,0,0,0,0,0]
dq_init = [0,0,0,0,0,0,0]
x_init = np.array([q_init, dq_init]).reshape((2*len(q_init), -1))
x_goal = np.array([q_goal, dq_goal]).reshape((2*len(q_init), -1))
# try:
#     X, U = bot1.ihlqr(x_init, x_goal,tau_goal, dt, Q, Qf, R, N, nx, nu)
# except Exception as e:
#     print("An error occurred:", e)
#     input("Press Enter to exit...")
M, N = bot1.computeM_N()
print("M", M)
print("N", N)
repls, reduced_exprs = sy.cse([M, N])
mass_matrix_reduced, gravity_term_reduced = reduced_exprs

[(py_filename, py_code), (py_header, header_code)] = codegen(
    name_expr=[("mass_matrix", mass_matrix_reduced),
               ("gravity_term", gravity_term_reduced)],
    language="Python",
    project="pybullet_sim",
    to_files=True  # set to False if you want to print code instead of writing to files
)


bot1.step_sim()