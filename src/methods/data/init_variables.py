
###Initialising variables
##aka don't touch these or program won't work
import numpy as np
from rolab_tb.turtlebot import Turtlebot
#Turtlebot instance
tb = Turtlebot() 

#booleans
RobotRunning = True
TimeTickUpdate_bool = False

#lists
timeFromStartArray = DeadReckon_List_theta = DeadReckon_List_vel = current_target = [0,0]
#Graph plotting CORY

IMUList = abscissaList = ordinateList = ordinateList2 = []

vErrorList = vTimeDifferences =aErrorList=aTimeDifferences = np.zeros(4) #each zero = 0.045sec

GlobalLoopCounter = 1

#floats    
#global forward_velocity
current_x = current_y =  distance_travelled= angle_total = 0
forward_velocity = global_velocity_signal=   0


