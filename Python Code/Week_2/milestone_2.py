# -*- coding: utf-8 -*-
"""
Created on Tue May  3 14:28:13 2022

@author: nagariku
"""

"""
testing
Possible need to get an average of last 10 values of angular velocity/forward velocity to calculate properly
move some things to only work when ticks are happening 
see if timedif and timedif2 are the same
make spin in correct direction
see about negative speed and it's impact on output signal 
same for angular velocity
"""

from distutils.log import error
from glob import glob
import time
from turtle import distance
from rolab_tb.turtlebot import Turtlebot
import numpy as np

#The variables
robotRunning = True
loopCounter = 0
refTickLeft = None
refTickRight = None
timeDifArray = [0,0]
newTimeTick = False





def tick_to_rad(val):
    # 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
    val = val * 0.001533981 # 2^12 = 4096 ticks/revolution
    return val

def get_linear_velocity():
    if timeDif != 0:
        vl = tick_to_rad(leftTick - refTickLeft) * 0.066 * 0.5/timeDif
        vr = tick_to_rad(rightTick - refTickRight) * 0.066 * 0.5/timeDif
        linear_vel = (vr + vl)/2
        
    else:
        linear_vel = 0
        
    return linear_vel

def get_angular_velocity():
    if timeDif != 0:
        
        vl = tick_to_rad(leftTick - refTickLeft) * 0.066 * 0.5/timeDif
        vr = tick_to_rad(rightTick - refTickRight) * 0.066 * 0.5/timeDif
        angu_vel = (vr-vl)/0.16 
    
    else:
        angu_vel = 0
    
    return angu_vel

def data_to_list(listToSave): 
    global timeDif2
    time2 = time.time()   
    timeDif = time2 - time1 
  
    timeDifArray.append(timeDif)
    timeDifArray.pop(0)
    timeDif2 = timeDifArray(1)-timeDifArray(0)
    if (timeDifArray(1)-timeDifArray(0))!=0:
        newTimeTick = True
    else:
        newTimeTick = False
    
    listToSave.append(tb.get_encoder_thicks())
    listToSave.append(timeDif)
    
    return listToSave

def get_current_theta():
    right_tick_relative = rightTick - refTickRight
    left_tick_relative = leftTick - refTickLeft
    difference = right_tick_relative - left_tick_relative
    if difference > 0:
        remainder = difference % 19859
        theta = remainder*0.0181274
    if difference < 0:
        remainder = (-difference) % (-19859)
        theta = remainder*(-0.0181274) 
        
    if difference == 0:
        theta = 0

    theta = theta/360*2*np.pi() #degrees to radians
    return theta

def get_xposition():
    global current_x,change_x
    if newTimeTick == True:
        change_x = forward_velocity*np.cos(theta)*(timeDif2)
        current_x = current_x + change_x
    else:
        change_x=0
    return current_x

def get_yposition():
    global current_y,change_y
    if newTimeTick == True:
        change_y = forward_velocity*np.sin(theta)*(timeDif2)
        current_y = current_x + change_y
    else:
        change_y= 0
    return current_y

def get_distance_moved():
    global change_x, change_y
    #if newTimeTick == True:
    distance_travelled = distance_travelled + np.sqrt(change_x^2 + change_y^2)
    return

def reach_correct_speed(set_LinVel):
    global vLastErr, vErrSum
    #Compute all the working error variables
    prop_error = set_LinVel - forward_velocity
    vErrSum = vErrSum + prop_error*timeDif2
    errDer = (prop_error-lastErr)/timeDif2
    #Compute PID Output
    out_signal = vKp * prop_error + vKi * vErrSum + errDer*vKd
    #Remember some variables for next time
    vLastErr = error
    if (out_signal>0.22):
        out_signal = 0.215
    tb.set_control_inputs(out_signal, 0) # set control input {lin-vel: 0.1, ang-vel:0}
    return

def reach_correct_angle(set_angle):
    global aLastErr, aErrSum
    #Compute all the working error variables
    prop_error = set_angle - theta
    aErrSum = vErrSum + prop_error*timeDif2
    errDer = (prop_error-lastErr)/timeDif2
    #Compute PID Output
    out_signal = aKp * prop_error + aKi * errSum + errDer*aKd
    #Remember some variables for next time
    aLastErr = error
    if (out_signal>0.22):
        out_signal = 0.215
    tb.set_control_inputs(0, out_signal) # set control input {lin-vel: 0.1, ang-vel:0}
    return

def reach_correct_distance(set_distance):
    global dLastErr, dErrSum
    #Compute all the working error variables
    prop_error = set_distance - distance_travelled
    dErrSum = vErrSum + prop_error*timeDif2
    errDer = (prop_error-lastErr)/timeDif2
    #Compute PID Output
    out_signal = aKp * prop_error + aKi * errSum + errDer*aKd
    #Remember some variables for next time
    dLastErr = error
    if (out_signal>0.22):
        out_signal = 0.215
    tb.set_control_inputs(out_signal, 0) # set control input {lin-vel: 0.1, ang-vel:0}
    return

def setVelTunings(input_Kp, input_Ki, input_Kd):
    global vKp, vKi, vKd
    vKp = input_Kp
    vKi = input_Ki
    vKd = input_Kd
    return    

def setAngleTunings(input_Kp, input_Ki, input_Kd):
    global aKp, aKi, aKd
    aKp = input_Kp
    aKi = input_Ki
    aKd = input_Kd
    return    

def setDistanceTunings(input_Kp, input_Ki, input_Kd):
    global dKp, dKi, dKd
    dKp = input_Kp
    dKi = input_Ki
    dKd = input_Kd
    return        

while robotRunning:

    if loopCounter == 0:
        returnedList = []
        tb = Turtlebot() 
        time1 = time.time()
        returnedList = data_to_list(returnedList)
        
        dataList = returnedList[0]
        timeDif = returnedList[1]
        
        current_x = 0
        current_y = 0

        change_x= 0
        change_y = 0

        vkp = 10
        vki = 0
        vkd = 0

        #aKp = -10
        aKi = 0
        aKd = 0

        dKp = -10
        dKi = 0
        dKd = 0

        vErrSum = 0
        vLastErr = 0

        aErrSum = 0
        aLstErr = 0

        dErrSum = 0
        dLastErr = 0

        distance_travelled =0
        
        refTickLeft = dataList['left']
        refTickRight = dataList['right']
        
        reach_correct_speed(0.04)
        #reach_correct_speed(np.pi())
        #reach_correct_distance(2)
        #tb.set_control_inputs(0.1, 0.1) # set control input {lin-vel: 0.1, ang-vel:0} 
        
    returnedList = []
    returnedList = data_to_list(returnedList)
    dataList = returnedList[0]
    timeDif = returnedList[1]

    leftTick = dataList['left']
    rightTick = dataList['right']
    
    forward_velocity = get_linear_velocity() 
    angular_velocity = get_angular_velocity() #* 57.29 # from radians to degrees
    distance_travelled = get_distance_moved()
    theta = get_current_theta()
    
    if loopCounter > 0:
        x_position = get_xposition()
        y_position = get_yposition()
    
    if loopCounter % 5 == 0:
        print("Linear velocity: ", str(round(forward_velocity, 5)))
        print("Angular velicity: ", str(round(angular_velocity, 5)))
        print("Angle: ", str(round(theta/2/np.pi*()*360, 5)))
        print("x position: ", str(round(x_position,5)))
        print("y position: ", str(round(y_position,5)))
            
    #previousTimeDif = timeDif
    
    loopCounter += 1
        
    if timeDif > 5:
        robotRunning = False
        
tb.stop()
        

    
