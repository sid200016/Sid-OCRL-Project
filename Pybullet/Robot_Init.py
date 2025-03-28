import pybullet as p
import pybullet_data
import time
import numpy as np
import autograd.numpy as anp
from autograd import jacobian
import sympy as sy
from sympy.utilities.codegen import codegen   

class Robot:
    def __init__(self, joint_angles_init, path):
        self.joint_angles_home = joint_angles_init
        self.path = path
        self.joint_init_positions = joint_angles_init
        
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        robotStartPos = [0,0,0]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0]) 
        self.robot_id = p.loadURDF(self.path,robotStartPos, robotStartOrientation)
        for joint_index in range(len(self.joint_init_positions)):
            p.resetJointState(self.robot_id, joint_index, self.joint_init_positions[joint_index])

    def getR_xT_x(self, q, p):
        R = sy.Matrix([[1, 0, 0],[0, sy.cos(q), -sy.sin(q)], [0, sy.sin(q), sy.cos(q)]])
        top = [R, p]
    # Bottom row: 1x3 zero matrix and 1x1 identity block
        bottom = [sy.Matrix([[0, 0, 0]]), sy.Matrix([[1]])]
    # Construct the BlockMatrix as a 2x2 grid of blocks
        T = sy.BlockMatrix([top, bottom])
        return R, T
    def getR_yT_y(self, q, p):
        R = sy.Matrix([[sy.cos(q), 0, sy.sin(q)],[0, 1, 0], [-sy.sin(q),0,  sy.cos(q)]])
        top = [R, p]
    # Bottom row: 1x3 zero matrix and 1x1 identity block
        bottom = [sy.Matrix([[0, 0, 0]]), sy.Matrix([[1]])]
    # Construct the BlockMatrix as a 2x2 grid of blocks
        T = sy.BlockMatrix([top, bottom])
        return R, T
    def getR_zT_z(self, q, p):
        R = sy.Matrix([[sy.cos(q), -sy.sin(q), 0],[sy.sin(q), sy.cos(q), 0], [0, 0, 1]])
        top = [R, p]
    # Bottom row: 1x3 zero matrix and 1x1 identity block
        bottom = [sy.Matrix([[0, 0, 0]]), sy.Matrix([[1]])]
    # Construct the BlockMatrix as a 2x2 grid of blocks
        T = sy.BlockMatrix([top, bottom])
        return R, T
    def skew2angvel(self, w_hat):
        w = sy.Matrix([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])
        return w
    def g2twist(self, Vhat):
        w_hat = Vhat[0:3, 0:3]
        w = self.skew2angvel(w_hat)
        v = Vhat[0:3, 3]
        V = sy.BlockMatrix([[v], [w]])
        return V
    def warp2pi(self, angle_rad):    
        angles =  ((angle_rad + np.pi) % (2 * np.pi)) - np.pi
        return angles
        
    def l1_transform(self):
        link_state = p.getLinkState(self.robot_id, 1  , computeForwardKinematics=True)
        return link_state
        #print(link_state)
    def get_transform_world(self, child_link):
        link_state = p.getLinkState(self.robot_id, child_link , computeForwardKinematics=True)
        pos = link_state[4]
        ori = link_state[5]
        rot_mat = np.array(p.getMatrixFromQuaternion(ori).reshape(3,3))
        T = np.eye(4)
        T[0:2, 0:2] = rot_mat
        T[0:2, 3] = pos  
        return T 
    def getMassMatrix(self, joint_pos):
        return np.array(p.calculateMassMatrix(self.robot_id, joint_pos)).reshape((7,7))
          
    def getCoriolisAndGrav(self, q, qdot):
        qdotdot=[0.0]*len(q)
        bias = p.calculateInverseDynamics(self.robot_id, q, qdot, qdotdot)
        return np.array(bias).reshape((7,1))
           
    def get_base_state(self):
        return p.getBasePositionAndOrientation(self.robot_id)
    def get_joint_states(self):
        num_joints = p.getNumJoints(self.robot_id)
        joint_angles = []
        for i in range(num_joints):
            joint_state = p.getJointState(self.robot_id, i)
            joint_angle = joint_state[0]  # The first element is the joint angle
           
            joint_angles.append(joint_angle)

        return joint_angles
    def step_sim(selfx):
        while True:
            p.stepSimulation()
            time.sleep(1/240)
    def computeM_N(self):
        #Define Parameters:
        q1, q2, q3, q4, q5, q6 = sy.symbols('q1 q2 q3 q4 q5 q6')
        dq1, dq2, dq3, dq4, dq5, dq6 = sy.symbols('dq1 dq2 dq3 dq4 dq5 dq6')
        q = sy.Matrix([[q1], [q2], [q3], [q4], [q5],[q6]])
        dq = sy.Matrix([[dq1], [dq2], [dq3], [dq4], [dq5],[dq6]])
        g = 9.81
        Rws = sy.eye(3)
        Tws = sy.Matrix([[1, 0, 0, 0], [0 , 1, 0, 0], [0, 0, 1,1], [0 ,0, 0, 1]])
        #Transforms relative to base (Just multiply by Tws to get everything in world frame.)
        #Base 
        pb = sy.Matrix([[0], [0], [0]])
        Rs, Ts = sy.eye(3), sy.eye(4)
        pb_com = sy.Matrix([[0], [0], [-0.0170]])
        Ts_com = sy.Matrix([[1, 0, 0, 0], [0 , 1, 0, 0], [0, 0, 1, -0.0170], [0 ,0, 0, 1]])
        Ts_com = Ts@Ts_com
        Twb_com = Tws@Ts_com
        print("j1")
        #joint 1
        psb = sy.Matrix([[0], [0], [0]])
        Rsb, Tsb = self.getR_zT_z(q1, psb)
        p1_com = sy.Matrix([[0], [0], [-0.0739]])
        Tb1_com = sy.Matrix([[1, 0, 0, 0], [0 , 1, 0, 0], [0, 0, 1, -0.0739], [0 ,0, 0, 1]])
        Ts1_com = (Tsb@Tb1_com)
        Ts1_com = sy.Matrix(Ts1_com)
        Twb = Tws@Tsb
        Tw1_com = Tws@Ts1_com 
        #joint 2
        print("j2")
        pb1 = sy.Matrix([[0], [0], [-0.1478]])
        Rb1, Tb1 = self.getR_yT_y(q2, pb1)
        p2_com = sy.Matrix([[0.13], [0], [0]])
        T12_com = sy.Matrix([[1, 0, 0, 0.13], [0 , 1, 0, 0], [0, 0, 1, 0], [0 ,0, 0, 1]])
        Ts2_com = (Tsb@Tb1@T12_com)
        Ts2_com = sy.Matrix(Ts2_com)
        Ts1 = sy.Matrix(Tsb@Tb1)
        Tw1 = Tws@Ts1
        Tw2_com = Tws@Ts2_com
        #joint 3
        print("j3")
        p12 = sy.Matrix([[0.26], [0], [0]])
        R12, T12 = self.getR_yT_y(q3, p12)
        p3_com = sy.Matrix([[0.13], [0], [0]])
        T23_com = sy.Matrix([[1, 0, 0, 0.13], [0 , 1, 0, 0], [0, 0, 1, 0], [0 ,0, 0, 1]]) 
        Ts2 = sy.Matrix(Ts1@T12)
        Ts3_com = (Tsb@Tb1@T12@T23_com)
        Ts3_com = sy.Matrix(Ts3_com)
        Tw2 = Tw1@T12
        Tw3_com = Tws@Ts3_com
        #Link 4
        print("j4")
        p23 = sy.Matrix([[0.26], [0], [0]])
        R23, T23 = self.getR_xT_x(q4, p23)
        p4_com = sy.Matrix([[0.0349], [0], [0]])
        T34_com = sy.Matrix([[1, 0, 0, 0.0349], [0 , 1, 0, 0], [0, 0, 1, 0], [0 ,0, 0, 1]]) 
        Ts3 = sy.Matrix(Ts2@T23)
        Ts4_com = (Tsb@Tb1@T12@T23@T34_com)
        Ts4_com = sy.Matrix(Ts4_com)
        Tw3 = Tw2@T23
        Tw4_com = Tws@Ts4_com
        print("j5")     
        #Link 5
        p34 = sy.Matrix([[0.0698], [0], [0]])
        R34, T34 = self.getR_zT_z(q5, p34)
        p5_com = sy.Matrix([[0.005], [0], [0]])
        T45_com = sy.Matrix([[1, 0, 0, 0.005], [0 , 1, 0, 0], [0, 0, 1, 0], [0 ,0, 0, 1]])
        Ts4 = sy.Matrix(Ts3@T34) 
        Ts5_com = (Tsb@Tb1@T12@T23@T34@T45_com)
        Ts5_com = sy.Matrix(Ts5_com)
        Tw4 = Tw3@T34
        Tw5_com = Tws@Ts5_com
        print("j6")
        #Link 6
        p45 = sy.Matrix([[0.01], [0], [0]])
        R45, T45 = self.getR_yT_y(q6, p45)
        p6_com = sy.Matrix([[0.013], [0], [0]])
        T56_com = sy.Matrix([[1, 0, 0, 0.013], [0 , 1, 0, 0], [0, 0, 1, 0], [0 ,0, 0, 1]])
        Ts6_com = (Tsb@Tb1@T12@T23@T34@T45@T56_com)
        Ts6_com = sy.Matrix(Ts6_com)
        Ts5 = sy.Matrix(Ts4@T45)
        Tw5 = Tw4@T45
        Tw6_com = Tws@Ts6_com
        print("EE")
        #End Effector
        p5E = sy.Matrix([[0.026], [0], [0]])
        T5e = sy.BlockMatrix([
        [sy.eye(3), p5E],
        [sy.Matrix([[0, 0, 0]]), sy.Matrix([[1]])]
        ])
        Tse = sy.Matrix(Ts5@T5e)
        Twe = Tw5@T5e
     
        #Mass Matrices
        #Base
        print("mass setup")
        M_Base = sy.diag(0.85, 0.85, 0.85, 0.00226, 0.00226, 0.0041388)
        #L1
        M_l1 = sy.diag(0.49, 0.49, 0.49, 0.000932, 0.00104, 0.000185)
        #L2
        M_l2  = sy.diag(0.99, 0.99, 0.99, 0.000375,0.00747, 0.00725)
        #L3
        M_l3 = sy.diag(0.98, 0.98, 0.98, 0.000375, 0.00738, 0.00715)
        #L4
        M_l4 = sy.diag(0.027, 0.027, 0.027, 4.322E-6, 4.317E-6, 2.702E-6)
        #L5
        M_l5 = sy.diag(0.0072, 0.0072, 0.0072, 4.807E-7, 5.229E-7, 5.229E-7)
        #L6
        M_l6 = sy.diag(0.027, 0.027, 0.027, 4.322E-6, 2.702E-6, 4.317E-6)
        #EE
        M_EE = sy.diag(0.047, 0.047, 0.047, 1.626E-5, 1.015E-5, 1.015E-6)
        #Jacobians:
        print("jacobi   ans")
        print("1")
        Ts1_com_inv =  Ts1_com.inv()
        print("2")
        Ts2_com_inv = Ts2_com.inv()
        print("3")  
        Ts3_com_inv = Ts3_com.inv()
        print("4")
        Ts4_com_inv = Ts4_com.inv()
        print("5")
        Ts5_com_inv = Ts5_com.inv()
        print("6")
        Ts6_com_inv = Ts6_com.inv()
        print("7")
        Tse_inv    = Tse.inv()
        print("Ts6_com:")
        sy.pprint(Ts6_com)
        print("Ts6_com_inv:")
        sy.pprint(Ts6_com_inv)
# Define a list of joint variables for convenience
        q_vars = [q1, q2, q3, q4, q5, q6]

# Compute each Jacobian block by looping over the joint variables
        Jb_sl1 = sy.BlockMatrix([
        self.g2twist(Ts1_com_inv @ sy.diff(Ts1_com, q_i)) for q_i in q_vars
])
        print("1 computed")

        Jb_sl2 = sy.BlockMatrix([
    self.g2twist(Ts2_com_inv @ sy.diff(Ts2_com, q_i)) for q_i in q_vars
])
        print("2 computed")

        Jb_sl3 = sy.BlockMatrix([
    self.g2twist(Ts3_com_inv @ sy.diff(Ts3_com, q_i)) for q_i in q_vars
])
        print("3 computed")

        Jb_sl4 = sy.BlockMatrix([
        self.g2twist(Ts4_com_inv @ sy.diff(Ts4_com, q_i)) for q_i in q_vars
])
        print("4 computed")

        Jb_sl5 = sy.BlockMatrix([
    self.g2twist(Ts5_com_inv @ sy.diff(Ts5_com, q_i)) for q_i in q_vars
])
        print("5 computed")

        Jb_sl6 = sy.BlockMatrix([
    self.g2twist(Ts6_com_inv @ sy.diff(Ts6_com, q_i)) for q_i in q_vars
])
        print("6 computed")

        Jbse = sy.BlockMatrix([
    self.g2twist(Tse_inv @ sy.diff(Tse, q_i)) for q_i in q_vars
])              
        print("7 computed")
        #Final Mass Matrix
        print("finals")
        M_fin = Jb_sl1.T@M_l1@Jb_sl1 + Jb_sl2.T@M_l2@Jb_sl2 + Jb_sl3.T@M_l3@Jb_sl3 + Jb_sl4.T@M_l4@Jb_sl4 + Jb_sl5.T@M_l5@Jb_sl5 + Jb_sl6.T@M_l6@Jb_sl6 + Jbse.T@M_EE@Jbse + M_Base
        print("M computed")
        N_fin = M_Base[0, 0]*g*pb_com[2, 1] + M_l1[0,0]*g*Ts1_com[2, 3] + M_l2[0,0]*g*Ts2_com[2, 3] + M_l3[0,0]*g*Ts3_com[2, 3] + M_l4[0,0]*g*Ts4_com[2, 3] + M_l5[0,0]*g*Ts5_com[2, 3] + M_l6[0,0]*g*Ts6_com[2, 3] + M_EE[0,0]*g*Tse[2, 3]
        print("N Computed")
        return M_fin, N_fin
    def computeC(self, M, q, qdot):
        l = M.shape[0]
        C = sy.zeros(l, l)
        for i in range(l):
            for j in range(l):
                for k in range(l):
                # Compute each element of the Coriolis matrix
                    C[i, j] += 0.5 * (M[i, j].diff(q[k]) + M[i, k].diff(q[j]) - M[k, j].diff(q[i])) * qdot[k]
        return C

    def forward_dynamics(self, x, tau):

        damper = 1e-4
        q_arr = x[0:7].reshape((7, -1))
        dq_arr = x[7:14].reshape((7, -1))
      
        #q = [float(np.array(val)) for val in q_arr.flatten()]
        #dq = [np.array(val).item() for val in q_arr.flatten()]
        q = q_arr._value
        dq = dq_arr._value
        
        M = self.getMassMatrix(q)+damper*np.eye(7)
        C_N = self.getCoriolisAndGrav(q, dq)
        C_N = np.array(C_N).reshape((7,-1))
        tau_arr = np.array(tau).reshape((7, -1))
        b = tau_arr - C_N
        L = np.linalg.cholesky(M)
        y = np.linalg.solve(L, b)

       
        ddq = np.linalg.solve(L.T, y)
        states_dot = np.concatenate((dq_arr, ddq), axis=0)
       
        return states_dot
    
    def rk4(self, x, tau, dt, nx, nu):
        q_arr = x[0:7].reshape((7, -1))
        dq_arr = x[7:14].reshape((7, -1))

        k1 = dt*self.forward_dynamics(x, tau)
    
        k2 = dt*self.forward_dynamics(x+ k1/2, tau)
     
        k3 = dt*self.forward_dynamics(x+k2/2, tau)
  
        k4 = dt*self.forward_dynamics(x+k3, tau)
        
        x_new = x+ (1/6)*(k1 + 2*k2 + 2*k3 + k4)
        x_new[0:nx//2] = self.warp2pi(x_new[0:nx//2])
        states = x_new
       
        return states
     
    def get_A_B(self, x_goal, u_goal, dt, nx, nu):
        print(nx//2)
        x_goal[0:nx//2] = self.warp2pi(x_goal[0:nx//2])
        print(x_goal)
        A = jacobian(lambda x: self.rk4(x, u_goal, dt, nx, nu))(x_goal)
        print(A)
        B = jacobian(lambda u: self.rk4(x_goal, u, dt, nx, nu))(u_goal)
        return A, B
        
    
    def ihlqr(self, x_init,x_goal,u_goal, dt, Q, Qf, R, N, nx, nu):
        tol = 1e-3
        A, B = self.get_A_B(x_goal, u_goal, dt, nx, nu)
        # Q = np.diag([1, 1, 1, 1,1,1, 1])
        # Qf = 10*Q 
        # R = np.diag([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        riccati_iter = 1000
        P = 1*Q
        K = np.zeros((riccati_iter, 1, nu))
        Xsim = np.zeros((N, nx, 1))
        Usim = np.zeros((N-1, nu, 1))
        Xsim[0] = x_init
      
        for i in range(riccati_iter):
            print("i, ", i)
            K[i] = np.linalg.inv(R + B.T@P@B) @ (B.T@P@A)
            P_new = Q + A.T@P@(A-B@K[i])
            #if np.linalg.matrix_norm(P_new -P, 2) < tol:
            #    break
            P = P_new
        K_ih = K[-1]
        print(K_ih)
        for j in range(N-1):
            print("j, ", j)
            u_k = -K_ih@(Xsim[j] - x_goal)
            Usim[j] = u_k
            x_j_1 = self.rk4(Xsim[j], u_k, dt, nx, nu)
            Xsim[j+1] = x_j_1
        print(Xsim)
        return Xsim, Usim
            

        
        

    

