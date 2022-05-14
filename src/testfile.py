#import time
#from rolab_tb.turtlebot import Turtlebot
#import numpy as np
##import matplotlib.pyplot as plt
#from scipy.interpolate import make_interp_spline
#import pandas as pd
#import json

from methods import actuators, getters, graph_plotting, p_controllers, pid_controllers, updaters
from methods.data import constants, init_variables

if __name__ == "__main__": 
    timeFromStart = 5
    getters.get_data_from_sensors(timeFromStart)

