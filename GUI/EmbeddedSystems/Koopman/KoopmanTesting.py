import numpy as np
from enum import Enum
import math
import pandas as pd

'''
Pseudocode:
- Prompt user to get the following information: 
    - Control inputs: x,y,z,g,j1,j2,j3
    - min and max for controls
    - position of object
    - Specify whether to sample the variable by a uniform or gaussian distribution. 
      Specify The mean and std.dev of the gaussian to vary x,y,z
    - Specify the sample hold time for each control input (T_h)
    - Specify length of time to collect data in minutes -> convert to seconds (T_total)
    - Generate random control trajectories, with length: T_Total/T_h
    - Run data collection for set length of time.
        - Every T_h seconds, send the next random step input. Log positions and pressures. Repeat until T_Total
'''

class variable_type(Enum):
    CONTROL = 1 #variable will be actively controlled
    STATE = 2 #variable will not be controlled

class sample_type(Enum):
    UNIFORM = 1  # sample values uniformly
    GAUSSIAN = 2  # sample values from a gaussian distribution


class var_params:

    def __init__(self, x_type = variable_type.STATE,
                 control_min_max=[0,20],
                 units='psi',
                 samp_type = sample_type.GAUSSIAN,
                 samp_parameters = {"mean":10, "stddev":3},
                 x_hold_time = 30,
                 total_run_time=300,
                 samp_freq_Hz = 20):

        self.var_type = x_type #Control variable or state variable
        self.control_min_max = control_min_max #only applicable for control variables. Min and max allowed
        self.units = units #units
        self.samp_type = samp_type #only applicable for control variables. Uniform or Gaussian
        self.samp_parameters = samp_parameters #dictionary with "mean" and "stddev" for gaussian sample type
        self.x_hold_time = x_hold_time #hold time in seconds to hold the control input
        self.total_run_time = total_run_time #total run time in seconds to run the experiment
        self.samp_freq_Hz = samp_freq_Hz #sampling frequency in Hz.
        self.random_sequence = [] #list to store the randomly generated sequence of controls

    def __str__(self):
        temp = "Variable Type: %s\n" \
               "control min and max: %f,%f\n" \
               "units: %s\n" \
               "Sample Type: %s\n" \
               "Sample Parameters: %s\n" \
               "Variable hold time: %f\n" \
               "Total run time: %f\n" \
               "Sample frequency in Hz: %f\n"%(self.var_type.name, self.control_min_max[0],self.control_min_max[1],
                                               self.units,self.samp_type.name, str(self.samp_parameters),
                                               self.x_hold_time,self.total_run_time,self.samp_freq_Hz)
        return (temp)


    def generate_random_sequence(self):

        num_points = math.floor(self.total_run_time*self.samp_freq_Hz) #number of points in the experiment
        num_control_points = math.floor(self.total_run_time*self.samp_freq_Hz/self.x_hold_time)
        num_points_episode = math.floor(num_points/num_control_points)
        s = np.zeros(num_points)

        match self.var_type:

            case variable_type.STATE:
                pass

            case variable_type.CONTROL:

                match self.samp_type:

                    case sample_type.UNIFORM:
                        cpoints = np.random.uniform(self.control_min_max[0],
                                              self.control_min_max[1],
                                              size = num_control_points)

                        s = np.array([np.repeat(i,num_points_episode)
                                      for i in cpoints]).flatten()


                    case sample_type.GAUSSIAN:
                        cpoints = np.random.normal(self.samp_parameters["mean"],
                                             self.samp_parameters["stddev"],
                                             size = num_control_points)

                        cpoints = np.clip(s,self.control_min_max[0], self.control_min_max[1])

                        s = np.array([np.repeat(i, num_points_episode)
                                      for i in cpoints]).flatten()



        self.random_sequence = s
        return (s)





class koopman:

    def __init__(self, var_defs=None,hold_time_s = 30, total_run_time_s = 1200, samp_freq_Hz = 20):
        if var_defs is None:
            var_defs = {"x": var_params(x_type=variable_type.CONTROL,
                                        control_min_max=[-12, 12],
                                        units='mm',
                                        samp_type=sample_type.UNIFORM,
                                        samp_parameters={},
                                        x_hold_time=hold_time_s,
                                        total_run_time=total_run_time_s,
                                        samp_freq_Hz=samp_freq_Hz),
                        "y": var_params(x_type=variable_type.CONTROL,
                                        control_min_max =[-12, 12],
                                        units='mm',
                                        samp_type = sample_type.UNIFORM,
                                        samp_parameters = {},
                                        x_hold_time = hold_time_s,
                                        total_run_time=total_run_time_s,
                                        samp_freq_Hz = samp_freq_Hz),
                        "z": var_params(x_type=variable_type.STATE,
                                        control_min_max=[0, 0],
                                        units='mm',
                                        samp_type=sample_type.UNIFORM,
                                        samp_parameters={},
                                        x_hold_time=hold_time_s,
                                        total_run_time=total_run_time_s,
                                        samp_freq_Hz=samp_freq_Hz),
                        "Grasper_Pressure": var_params(x_type=variable_type.CONTROL,
                                        control_min_max=[0, 11.2],
                                        units='psi',
                                        samp_type=sample_type.UNIFORM,
                                        samp_parameters={},
                                        x_hold_time=hold_time_s,
                                        total_run_time=total_run_time_s,
                                        samp_freq_Hz=samp_freq_Hz),
                        "Jaw1": var_params(x_type=variable_type.STATE,
                                        control_min_max=[0, 30],
                                        units='psi',
                                        samp_type=sample_type.UNIFORM,
                                        samp_parameters={},
                                        x_hold_time=hold_time_s,
                                        total_run_time=total_run_time_s,
                                        samp_freq_Hz=samp_freq_Hz),
                        "Jaw2": var_params(x_type=variable_type.STATE,
                                        control_min_max=[0, 30],
                                        units='psi',
                                        samp_type=sample_type.UNIFORM,
                                        samp_parameters={},
                                        x_hold_time=hold_time_s,
                                        total_run_time=total_run_time_s,
                                        samp_freq_Hz=samp_freq_Hz),
                        "Jaw3": var_params(x_type=variable_type.STATE,
                                           control_min_max=[0, 30],
                                           units='psi',
                                           samp_type=sample_type.UNIFORM,
                                           samp_parameters={},
                                           x_hold_time=hold_time_s,
                                           total_run_time=total_run_time_s,
                                           samp_freq_Hz=samp_freq_Hz)
                        }

        self.vars = var_defs


    def compute_variable_sequence(self):
        for (k,v) in self.vars.items():
            v.generate_random_sequence()
            print(k)
            print(v.random_sequence)



if __name__ == '__main__':
    kpm = koopman()
    kpm.compute_variable_sequence()
    print(kpm.vars["y"])



