# -*- coding: utf-8 -*-
"""
Created on Mon Apr 25 10:36:41 2022

@author: admin

setting velocity to 0.22 or more (max): robot doesn't accept it

FIX: time function not working, find time
Obtained times:
1650884195.7843752
1650884195.7999966
0.015621423721313477
1650884195.7999966
1650884195.7999966
0.0
1650884195.7999966
1650884195.7999966
0.0
=> forgot to increase the counter every loop
"""

import time
from rolab_tb.turtlebot import Turtlebot

tb = Turtlebot() 
running = True # create turtlebot instance

counter = 0
refList1 = []
while True:
    if counter == 0:
        time1 = time.time()
        tb.set_control_inputs(0.2, 0) # set control input {lin-vel: 0.1, ang-vel:0}  
    
    tempList = []
    time2 = time.time()
    print(time1)
    print(time2)
    timeDif = time2 - time1
    print(timeDif)
    tempList = [tb.get_tick_to_rad(), timeDif]
    refList1.append(tempList)
    tempList = []
    
    if timeDif > 4 and timeDif < 5:
        print("Time of ", str(timeDif), " was reached")
        tb.stop()
        break
    
    counter += 1
    
    
counter = 0
refList2 = []
while True:
    if counter == 0:
        time1 = time.time()
        tb.set_control_inputs(0, -2.8) # set control input {lin-vel: 0.1, ang-vel:0}  
        
    tempList = []
    time2 = time.time()
    timeDif = time2 - time1
    tempList = [tb.get_tick_to_rad(), timeDif]
    #tempList = [tb.get_encoder_thicks(),tb.get_tick_to_rad(), timeDif]
    refList2.append(tempList)
    tempList = []
    
    if timeDif > 4 and timeDif < 5:
        print("Time of ", str(timeDif), " was reached")
        tb.stop()
        break
    
print(refList1)
print(refList2)
    





    