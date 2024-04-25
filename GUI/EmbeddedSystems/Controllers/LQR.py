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
        self.target_trajectory = []
        self.u_feedforward = []
        

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

        i.e. there should be nested lists where each list is a row in the matrix

        """


        with open(fname) as uf:
            fc = uf.read()
            pc = json.loads(fc)


        self.A = np.array(pc["A"])
        self.B = np.array(pc["B"])
        self.C = np.array(pc["C"])
        self.K = [np.array(pc["K"][i]) for i in range(0,len(pc["K"]))]
        self.target_trajectory = np.array(pc["target_trajectory"])
        self.u_feedforward = [np.array(pc["u_feedforward"][i]) for i in range(0,len(pc["u_feedforward"]))]

if __name__ == "__main__":
    lq = LQR()
    lq.read_from_JSON()
    pass
