import time
#from rolab_tb.turtlebot import Turtlebot
#import numpy as np
##import matplotlib.pyplot as plt
#from scipy.interpolate import make_interp_spline
#import pandas as pd
import json

from methods import actuators, getters, graph_plotting, p_controllers, pid_controllers, updaters
import data.pid_variables
import data.init_variables

actuators.reach_forward_speed(5)
data.init_variables.forward_velocity = 4 
import data.init_variables
actuators.reach_forward_speed(5)