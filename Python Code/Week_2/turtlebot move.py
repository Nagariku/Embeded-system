# -*- coding: utf-8 -*-
"""
Created on Mon Apr 25 10:36:41 2022

@author: admin

setting velocity to 0.22 or more (max): robot doesn't accept it

Note:
    Code stops reading when tb.stop() called
    Linear: positive & negative works
    Test 1: Time of  3.000976324081421 s was reached, when >3 used
    All data: [tb.get_encoder_thicks(),tb.get_tick_to_rad(), tb.get_imu(), tb.get_imu_angle(), timeDif]
    Essential data: imu_angle and encoder ticks and time

"""

import time
from rolab_tb.turtlebot import Turtlebot
import pandas as pd
import numpy as np

def data_to_list(listToSave):
    time2 = time.time()
    timeDif = time2 - time1
    # SAVE ALL
    #tempList = [tb.get_encoder_thicks(),tb.get_tick_to_rad(), tb.get_imu(), tb.get_imu_angle(), timeDif]
    
    # SAVE ESSENTIAL
    tempList = [tb.get_encoder_thicks(), timeDif]
    
    listToSave.append(tempList)
    return listToSave, timeDif
    
def data_to_csv(listToSave):
    name = "Saved data angular v2"+ ".csv" # change name according to recording type
    dict = {'Recieved Locations': listToSave}
    df = pd.DataFrame(dict)
    df.to_csv(name)
    np.savetxt(name, listToSave, delimiter =", ", fmt ='% s')

tb = Turtlebot() 

counter = 0
dataList = []
loop1Go = True

######################################## PART 1 ########################################
# Does: 
#    run linear 5s, reponding to a step velocity command of 0.2, saves ALL AND essential (run twice)
# while loop1Go:
#     if counter == 0:
#         print("---- LOOP ENTERED ----")
#         time1 = time.time()
#         tb.set_control_inputs(0.1, 0) # set control input {lin-vel: 0.1, ang-vel:0}  
    
#     returnedList = data_to_list(dataList)
#     dataList = returnedList[0]
#     timeDif = returnedList[1]
    
#     if timeDif > 9.5:
#         print("Time of ", str(timeDif), " was reached")
#         tb.set_control_inputs(0, 0)
#         loop1Go = False
    
#     counter += 1
    
# print("Linear velocity ",dataList)

###################################### PART 2 ######################################
# Does: 
#    run angular 5s, reponding to a step velocity command of 2.5, saves ALL AND essential (run twice)

counter = 0
loop2Go = True
refList2 = []

while loop1Go:
    if counter == 0:
        print("---- LOOP ENTERED ----")
        time1 = time.time()
        #tb.set_control_inputs(0, 2.5) # set control input {lin-vel: 0.1, ang-vel:0}  
        tb.set_control_inputs(0, 1.25)
    
    returnedList = data_to_list(dataList)
    dataList = returnedList[0]
    timeDif = returnedList[1]
    
    if timeDif > 9.5:
        print("Time of ", str(timeDif), " was reached")
        tb.set_control_inputs(0, 0)
        loop1Go = False
    
    counter += 1
    
print("Angular velocity ",dataList)


data_to_csv(dataList)
tb.stop()
    





    