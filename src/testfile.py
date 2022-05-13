import time
#from rolab_tb.turtlebot import Turtlebot
#import numpy as np
##import matplotlib.pyplot as plt
#from scipy.interpolate import make_interp_spline
#import pandas as pd
import json

from methods import actuators, getters, graph_plotting, p_controllers, pid_controllers, updaters
from data import pid_variables #glob_variables, #init_variables, 

actuators.reach_forward_speed(5)
