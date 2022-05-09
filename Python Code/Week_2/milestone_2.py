# -*- coding: utf-8 -*-
"""
Created on Tue May  3 14:28:13 2022

@author: nagariku
"""

"""
testing
Possible need to get an average of last 10 values of angular velocity/forward velocity to calculate properly
move some things to only work when ticks are happening 
see if timedif and timedif2 are the same: i belive they are not the same
make spin in correct direction
see about negative speed and it's impact on output signal 
same for angular velocity
Corentin:
Limitation of current controller: cannot move linearly and turn simultaneously (see def reach_correct_speed(set_LinVel) and other)
"""

from distutils.log import error
from glob import glob
import time
from turtle import distance
from rolab_tb.turtlebot import Turtlebot
import numpy as np

#The variables
robotRunning = True
newTimeTick = False

newTimeTick = False
refTickLeft = None
refTickRight = None

timeDifArray = [0,0]
thetaDeadReckon = [0,0]
vDeadReckon = [0,0]

loopCounter = 0


def tick_to_rad(val):
    '''
    Parameter:
    Tick reading
    -----
    Action: 
    Converts the encoders ticks into radians (multiplication factor provided)
    -----
    Returns: 
    The radian equivalent
    '''
    # 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
    val = val * 0.001533981 # 2^12 = 4096 ticks/revolution
    return val

def get_linear_velocity():
    '''
    Parameter:
    None
    -----
    Action: 
    Calculates from the tick reading and the recorded time the bot velocity
    -----
    Returns: 
    The linear velocity of the platform center point (mid point between wheels)
    '''
    #global vDeadReckon
    if timeDif != 0:
        vl = tick_to_rad(leftTick - refTickLeft) * 0.066 * 0.5/timeDif
        vr = tick_to_rad(rightTick - refTickRight) * 0.066 * 0.5/timeDif
        linear_vel = (vr + vl)/2
        
    else:
        linear_vel = 0

    if newTimeTick ==True:
        vDeadReckon.append(linear_vel)
        vDeadReckon.pop(0)
        
    return linear_vel

def get_angular_velocity():
    '''
    Parameter:
    None
    -----
    Action: 
    Calculates from the tick reading and the recorded time the bot velocity
    -----
    Returns: 
    The linear velocity of the platform center point (mid point between wheels)
    '''
    if timeDif != 0:
        
        vl = tick_to_rad(leftTick - refTickLeft) * 0.066 * 0.5/timeDif
        vr = tick_to_rad(rightTick - refTickRight) * 0.066 * 0.5/timeDif
        angu_vel = (vr-vl)/0.16 
    
    else:
        angu_vel = 0
    
    return angu_vel

def data_to_list(listToSave):
    '''
    Parameter:
    List to be extended
    -----
    Action: 
    Appends the recieved list with the tick reading and the time passed since start of the robot
    -----
    Returns: 
    The extended list
    '''
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
    '''
    Parameter:
    None
    -----
    Action: 
    Calculates the theta angle the robot perfomed according to the reference frame (taken at the robot start position)
    -----
    Returns: 
    The live theta angle
    '''
    global thetaDeadReckon
    right_tick_relative = rightTick - refTickRight
    left_tick_relative = leftTick - refTickLeft
    difference = right_tick_relative - left_tick_relative
    if difference > 0:
        remainder = difference % 19859
        theta = remainder*0.0181274
    if difference < 0:
        remainder = (-difference) % (-19859)
        theta = remainder*(-0.0181274) 
    else:
        theta = 0
    if newTimeTick ==True:
        thetaDeadReckon.append(theta)
        thetaDeadReckon.pop(0)
    theta = theta/360*2*np.pi() #degrees to radians
    return theta

def get_xposition():
    ''' 
    Parameter:
    None
    -----
    Action: 
    Calculates the live x coordinate of the robot from the linear velocity and the theta angle
    -----
    Returns: 
    The x coordinate of the platform center point (mid point between wheels)
    '''
    global current_x,change_x
    if newTimeTick == True:
        thetaAverageTick = thetaDeadReckon(1)-thetaDeadReckon(0)
        vAverageTick = vAverageTick(1)-vAverageTick(0)
        change_x = vAverageTick*np.cos(thetaAverageTick)*(timeDif2) # error appearing when speed is not constant
        current_x = current_x + change_x
    else:
        change_x = 0
    return current_x

def get_yposition():
    '''
    Parameter:
    None
    -----
    Action: 
    Calculates the live y coordinate of the robot from the linear velocity and the theta angle
    -----
    Returns: 
    The y coordinate of the platform center point (mid point between wheels)
    '''
    global current_y,change_y
    if newTimeTick == True:
        thetaAverageTick = thetaDeadReckon(1)-thetaDeadReckon(0)
        vAverageTick = vAverageTick(1)-vAverageTick(0)
        change_y = vAverageTick*np.sin(thetaAverageTick)*(timeDif2) # error appearing when speed is not constant
        current_y = current_y + change_y
    else:
        change_y= 0
    return current_y

def get_distance_moved():
    '''
    Parameter:
    None
    -----
    Action: 
    Calculates distance traveled, adding the euclidian distance of the last change in position to the total traveled distance
    -----
    Returns: 
    None
    '''
    global change_x, change_y # Unecessary, global is needed only when the variables are changed within the function
    #if newTimeTick == True:
    distance_travelled = distance_travelled + np.sqrt(change_x^2 + change_y^2) # Euclidian distance assumes the distance traveled is the shortest one (no curves, turns etc)
    return None

def reach_correct_speed(set_LinVel):
    '''
    Parameter:
    The linear velocity set, aka velocity in the x axis (in the robot frame)
    -----
    Action: 
    Sets the linear velocity, that is calculated from the determined error and the PID parameters
    -----
    Returns: 
    None
    '''
    global vLastErr, vErrSum
    #Compute all the working error variables
    prop_error = set_LinVel - forward_velocity
    vErrSum = vErrSum + prop_error*timeDif2
    errDer = (prop_error-vLastErr)/timeDif2
    #Compute PID Output
    out_signal = vKp * prop_error + vKi * vErrSum + errDer*vKd
    #Remember some variables for next time
    vLastErr = error
    if (out_signal>0.22):
        out_signal = 0.215
    tb.set_control_inputs(out_signal, 0) # set control input {lin-vel: out_signal, ang-vel:0}
    return None

def reach_correct_angle(set_angle):
    '''
    Parameter:
    The angular velocity set, aka angular in the theta axis (in the robot frame)
    -----
    Action: 
    Sets the angular velocity, that is calculated from the determined error and the PID parameters
    -----
    Returns: 
    None
    '''
    global aLastErr, aErrSum
    #Compute all the working error variables
    prop_error = set_angle - theta
    aErrSum = vErrSum + prop_error*timeDif2
    errDer = (prop_error-aLastErr)/timeDif2
    #Compute PID Output
    out_signal = aKp * prop_error + aKi * aErrSum + errDer*aKd
    #Remember some variables for next time
    aLastErr = error
    if (out_signal>0.22):
        out_signal = 0.215
    tb.set_control_inputs(0, out_signal) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_correct_distance(set_distance):
    '''
    Parameter:
    Target coordinate
    -----
    Action: 
    ????????????
    Corentin doesn't get its purpose, same as reach_correct_speed(set_LinVel)?
    -----
    Returns: 
    None
    '''
    global dLastErr, dErrSum
    #Compute all the working error variables
    prop_error = set_distance - distance_travelled
    dErrSum = vErrSum + prop_error*timeDif2
    errDer = (prop_error-dLastErr)/timeDif2
    #Compute PID Output
    out_signal = aKp * prop_error + aKi * dErrSum + errDer*aKd
    #Remember some variables for next time
    dLastErr = error
    if (out_signal>0.22):
        out_signal = 0.215
    tb.set_control_inputs(out_signal, 0) # set control input {lin-vel: out_signal, ang-vel:0}
    return None

def setVelTunings(input_Kp, input_Ki, input_Kd):
    '''
    Parameter:
    The PID parameters for the velocity controller
    -----
    Action: 
    Sets the PID parameters
    -----
    Returns: 
    None
    '''
    global vKp, vKi, vKd
    vKp = input_Kp
    vKi = input_Ki
    vKd = input_Kd
    return None

def setAngleTunings(input_Kp, input_Ki, input_Kd):
    '''
    Parameter:
    The PID parameters for the angular velocity controller
    -----
    Action: 
    Sets the PID parameters
    -----
    Returns: 
    None
    '''
    global aKp, aKi, aKd
    aKp = input_Kp
    aKi = input_Ki
    aKd = input_Kd
    return    

def setDistanceTunings(input_Kp, input_Ki, input_Kd):
    '''
    Parameter:
    The PID parameters for the ??? controller
    -----
    Action: 
    Sets the PID parameters
    -----
    Returns: 
    None
    '''
    global dKp, dKi, dKd
    dKp = input_Kp
    dKi = input_Ki
    dKd = input_Kd
    return None       

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
        

    
