import h5py
import numpy as np
import json



class LQR():
    """
    Class LQR
    For tracking trajectories on a system with linear dynamics in state-space form:
    z[k+1] = A*z[k] + B*u[k]
    p[k] = C*z[k] + D*u[k]

    where:
    p are the observables [np,1]
    z is the state [nz,1]
    u are the controls [nu,1]
    A is the state matrix [nz,nz]
    B is the input matrix [nz,nu]
    C is the output matrix [np,nz]
    D is the feedthrough matrix [np,nu]

    LQR will produce a control law of the following form to track
    a desired trajectory p_des that is N timesteps long with feedforward control u_ff and current state p :

    for k=1:N:
        z_des = map_p_to_z(p_des[k])
        z[k] = map_z_to_p(p[k])
        u[k] = u_ff - K*(z[k] - z_des)
        z[k+1] = dynamics(u[k],z[k]) #advance to the next state
    end
    """

    def __init__(self):
        self.K = []
        self.A = []
        self.B = []
        self.C = []
        self.C_inv = []
        self.target_trajectory = []
        self.u_feedforward = []
        self.N_steps = 0
        

    def read_from_JSON(self,
                       fname = "C://Users//Ravesh//Desktop//Courses//16745 Optimal Control//16745 Course Project//16745_CourseProject//LQR_output.json"):
        """
        read_from_JSON

        Function: populate the K, A, B, C, target_trajectory and u_feedforward from a JSON file.
        For a matrix like B, which has dimensions nz x nu and a structure like the following:

        B = []

        it should have the following structure in the JSON file:

        B = [a_1, b_1, c_1
             a_2, b_2, c_2
             .     .    .
             .     .    .
             .     .    .
             a_z, b_z, c_z]

        "B" :[
            [
                a_1,
                b_1,
                c_1
            ],
            [
                a_2,
                b_2,
                c_2
            ],
            .
            .
            .
            [
                a_z,
                b_z,
                c_z
            ]]

        i.e. there should be nested lists where each list is a row in the matrix.

        A: nz x nz
        B: nz x nu
        C: np x nz
        K: list of numpy arrays (nu x nz) (Note: length is N-1, if N is the number of timesteps)
        target_trajectory: np x N (each column is the desired outputs)
        u_feedforward: list of numpy arrays (3 x 1)
        """


        with open(fname) as uf:
            fc = uf.read()
            pc = json.loads(fc)


        self.A = np.array(pc["A"])
        self.B = np.array(pc["B"])
        self.C = np.array(pc["C"])
        self.C_inv = np.linalg.pinv(self.C)
        self.K = [np.array(pc["K"][i]) for i in range(0,len(pc["K"]))]
        self.target_trajectory = np.array(pc["target_trajectory"])
        self.u_feedforward = [np.array(pc["u_feedforward"][i]) for i in range(0,len(pc["u_feedforward"]))]
        self.N_steps = np.size(self.target_trajectory,1)

    def calculate_control(self,current_output : np.ndarray, time_step_int : int):
        """
        calculate_control
        For a problem that is:
        z[k+1] = A*z[k] + B*u[k]
        p[k] = C*z[k] + D*u[k]

        At current time_step k, calculate the control inputs u[k] using the LQR control for the current output p[k] and feedforward u_feedforward[k]:
        u_k = u_feedforward[k] - K[k]*(z[k]-z_target[k])

        Parameters
        ----------
        current_output: np.ndarray, npx1 vector
        time_step_int: integer representing the timestep, (1:N-1)

        Returns
        -------
        u_k
        """
        z_k = self.map_p_to_z(current_output)
        z_target_k = self.map_p_to_z(self.target_trajectory[:,time_step_int])
        u_k = self.u_feedforward[time_step_int] - self.K[time_step_int]@(z_k - z_target_k)
        return(u_k)

    def map_p_to_z(self , p):
        z = self.C_inv@p
        return z

    def map_z_to_p(self , z):
        p = self.C@z
        return(p)





if __name__ == "__main__":
    lq = LQR()
    lq.read_from_JSON()
    print("Calculated Controls:")
    print(lq.calculate_control(np.array([0.06,0.015,0.03]), 0))
    print("\nTarget Trajectory:")
    print(lq.target_trajectory[:,0])
    print("\nFeedforward controls:")
    print(lq.u_feedforward[0])
    pass
