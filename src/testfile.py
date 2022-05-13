import time
#from rolab_tb.turtlebot import Turtlebot
import numpy as np
##import matplotlib.pyplot as plt
#from scipy.interpolate import make_interp_spline
#import pandas as pd
import json

from methods import actuators, getters, graph_plotting, p_controllers, pid_controllers, updaters
forward_velocity = 0
actuators.reach_forward_speed(5)
forward_velocity = 4 
actuators.reach_forward_speed(5)