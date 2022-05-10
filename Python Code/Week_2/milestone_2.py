# -*- coding: utf-8 -*-
"""
Created on Tue May  3 14:28:13 2022

@author: nagariku
"""

"""
testing
Possible need to get an average of last 10 values of angular velocity/forward velocity to calculate properly
move some things to only work when ticks are happening 
see if timedif and timedif2 are the same: i believe they are not the same
make spin in correct direction
see about negative speed and it's impact on output signal 
same for angular velocity
"""

from distutils.log import error
import time
from rolab_tb.turtlebot import Turtlebot
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import pands as pd
import json

#The variables
robotRunning = True
newTimeTick = False
targetTReached = False

newTimeTick = False
refTickLeft = None
refTickRight = None

timeDifArray = [0,0]
thetaDeadReckon = [0,0]
vDeadReckon = [0,0]
loopCounter = 1

def data_to_csv(listToSave, csvName):
    """
    Works: but watch out with csv file name when rrunning the cod emultiple times

    """
    name = csvName + ".csv" #"Saved data angular"+ ".csv" # change name according to recording type
    dict = {name: listToSave}
    df = pd.DataFrame(dict)
    df.to_csv(name)
    np.savetxt(name, listToSave, delimiter =", ", fmt ='% s')

def plot_this(abscissaList, ordinateList):
    """    
    Works

    """
    abscissaList = np.asarray(abscissaList)
    ordinateList = np.asarray(ordinateList)
    B_spline_coeff = make_interp_spline(abscissaList, ordinateList)
    X_Final = np.linspace(abscissaList.min(), abscissaList.max(),75) #choose the resolution as 5010
    Y_Final = B_spline_coeff(X_Final)

    plt.plot(X_Final, Y_Final)
    plt.ylabel('Distance to wall (m)') #set the label for y axis
    plt.xlabel('Time (s)') #set the label for x-axis
    plt.title("Wall proximity") #set the title of the graph
    plt.grid()
    plt.show() #display the graph

def get_nearest_object():
    """
    Works
    """

    data = tb.get_scan()
    data = json.loads(data) # converts string into dictionnary
    rangesList = data["ranges"]
    maxRange = data["range_max"]
    minRange = data["range_min"]
    closest_ting = maxRange    
    
    for i in rangesList:
        if i != None:
            if (i < closest_ting) and (i > minRange):
                closest_ting = i
            
    return closest_ting


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
    global timeDif2, newTimeTick
    time2 = time.time()   
    timeDif = time2 - time1 
  
    timeDifArray.append(timeDif)
    timeDifArray.pop(0)
    timeDif2 = timeDifArray[1]-timeDifArray[0]
    
    if (timeDifArray[1]-timeDifArray[0])!=0:
        newTimeTick = True
        vDeadReckon.append(forward_velocity)
        vDeadReckon.pop(0)
        thetaDeadReckon.append(theta)
        thetaDeadReckon.pop(0)
        update_angle_total()
        update_distance_moved()
        
    else:
        newTimeTick = False
    listToSave.append(tb.get_encoder_ticks())
    listToSave.append(timeDif)
    
    return listToSave

def get_current_theta():
    global thetaDeadReckon,theta
    right_tick_relative = rightTick - refTickRight
    left_tick_relative = leftTick - refTickLeft
    difference = right_tick_relative - left_tick_relative
    if difference > 0:
        remaindery = difference % 19859
        theta = remaindery*0.0181274
        theta = theta/360*2*np.pi #degrees to radians
    if difference < 0:
        remaindery = (-difference) % (-19859)
        theta = remaindery*(-0.0181274) 
        theta = theta/360*2*np.pi #degrees to radians
    else:
        theta = 0
    return theta

def get_averagesVandAngle():
    #correction to prevent negative angle fuckery
    averageSpeed = (vDeadReckon[1]+vDeadReckon[0])/2
    averageTheta = (thetaDeadReckon[1] + thetaDeadReckon[0])/2
    if averageTheta >100:
        biggerTheta = -360+max(thetaDeadReckon)
        smallerTheta = min(thetaDeadReckon)
        averageTheta = (biggerTheta + smallerTheta)/2
        if averageTheta<0:
            return averageSpeed, 360+averageTheta,
    return averageSpeed, averageTheta

def get_distance_to_coordinate(listCoordsInput):
    deltaX=current_x - listCoordsInput[0]
    deltaY= current_y - listCoordsInput[1]
    totDist = np.sqrt(deltaY**2+deltaX**2)
    return totDist

def get_distance_to_wall():
    return wall_y_coordinate-current_y

def get_angle_to_next_coord(coord_x,coord_y):
    relative_x = coord_x - current_x
    relative_y = coord_y - current_y
    if (relative_x>0):
        if (relative_y>0):    
            desired_angle = np.arctan((relative_y)/(relative_x))
        elif (relative_y==0):
            desired_angle = 0
        else:
            desired_angle = 2*np.pi+np.arctan((relative_y)/(relative_x))
    elif (relative_x<0):
        if (relative_y>0):
            desired_angle = np.pi+np.arctan((relative_y)/(relative_x))
        elif (relative_y==0):
            #desired_angle = 0
            #constSpeed = -constSpeed
            desired_angle = np.pi
        else:
            desired_angle = np.pi+np.arctan((relative_y)/(relative_x))
    else:
        if (relative_y>0):
            desired_angle = np.pi/2
        elif (relative_y==0):
            desired_angle = 0
        else:
            desired_angle = 3*np.pi/2
    return desired_angle

def update_distance_moved():
    global distance_travelled # Unecessary, global is needed only when the variables are changed within the function
    if (newTimeTick == True and (change_x!=0 or change_y!=0)):
        distance_travelled = distance_travelled + np.sqrt(change_x**2 + change_y**2) # Euclidian distance assumes the distance traveled is the shortest one (no curves, turns etc)
    return None

def update_angle_total():
    global angle_total
    if (max(thetaDeadReckon())-min(thetaDeadReckon))>1: # sensetivity
        biggerTheta = -2*np.pi+max(thetaDeadReckon)
        smallerTheta = min(thetaDeadReckon)
        angle_total = angle_total + smallerTheta - biggerTheta
    else:
        angle_total = angle_total + (max(thetaDeadReckon())-min(thetaDeadReckon)) # Euclidian distance assumes the distance traveled is the shortest one (no curves, turns etc)
    return None

def update_xposition():
    global current_x,change_x
    if newTimeTick == True:
        vAverageTick, thetaAverageTick = get_averagesVandAngle()
        change_x = vAverageTick*np.cos(thetaAverageTick)*(timeDif2) # error appearing when speed is not constant
        current_x = current_x + change_x
    else:
        change_x = 0
    return current_x

def update_yposition():
    global current_y,change_y
    if newTimeTick == True:
        vAverageTick, thetaAverageTick = get_averagesVandAngle()
        change_y = vAverageTick*np.sin(thetaAverageTick)*(timeDif2) # error appearing when speed is not constant
        current_y = current_y + change_y
    else:
        change_y= 0
    return current_y

def reach_correct_speed(set_LinVel):
    global vLastErr, vErrSum
    #Compute all the working error variables
    prop_error = set_LinVel - forward_velocity
    vErrSum = vErrSum + prop_error*timeDif2
    errDer = (prop_error-vLastErr)/timeDif2
    #Compute PID Output
    out_signal = vKp * prop_error + vKi * vErrSum + errDer*vKd
    #Remember some variables for next time
    vLastErr = error
    final_signal = set_LinVel+out_signal
    if (final_signal >0.22):
        final_signal  = 0.215
    tb.set_control_inputs(final_signal, 0) # set control input {lin-vel: out_signal, ang-vel:0}
    return None

def reach_correct_angle(set_angle):
    outInFuncSignal = reach_correct_angle_signal(set_angle)
    tb.set_control_inputs(0, outInFuncSignal) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_correct_distance(set_distance):
    global dLastErr, dErrSum
    #Compute all the working error variables
    prop_error = set_distance - distance_travelled
    if (timeDif2 != 0):
        dErrSum = dErrSum + prop_error*timeDif2
        errDer = (prop_error-dLastErr)/timeDif2
    #Compute PID Output
    out_signal = dKp * prop_error + dKi * dErrSum + errDer*dKd
    #Remember some variables for next time
    dLastErr = prop_error
    if (out_signal>0.22):
        out_signal = 0.215
    tb.set_control_inputs(out_signal, 0) # set control input {lin-vel: out_signal, ang-vel:0}
    return None

def reach_correct_angle_total(set_angle_total):
    global aLastErr, aErrSum
    #Compute all the working error variables
    prop_error = set_angle_total - angle_total
    aErrSum = aErrSum + prop_error*timeDif2
    errDer = (prop_error-aLastErr)/timeDif2
    #Compute PID Output
    out_signal = aKp * prop_error + aKi * aErrSum + errDer*aKd
    #Remember some variables for next time
    aLastErr = error
    if (out_signal>2.84):
        out_signal = 2.795
    tb.set_control_inputs(0, out_signal) # set control input {lin-vel: out_signal, ang-vel:0}
    # MAKE IT STOP ON TIME
    return None

def reach_correct_angle_signal(set_angle):
    global aLastErr, aErrSum
    #Compute all the working error variables
    prop_error = set_angle - theta

    #deciding direction of angV
    if (prop_error ==0 or np.pi):
        turnRight = 1
        #cause why not, no need to make it an RNG
    if (set_angle>np.pi):
        bigOpposite = set_angle-np.pi
        if (theta>bigOpposite and theta<set_angle):
            turnRight = 1
        else:
            turnRight = -1
    else:
        smallOpposite = set_angle + np.pi
        if (theta>set_angle and theta<smallOpposite):
            turnRight = -1
        else:
            turnRight= 1

    if prop_error > np.pi:
        correct_prop_error = 2*np.pi-prop_error
    if prop_error < 0:
        correct_prop_error = 2*np.pi+prop_error

    aErrSum = aErrSum + correct_prop_error*timeDif2
    errDer = (correct_prop_error-aLastErr)/timeDif2
    #Compute PID Output
    out_signal = aKp * correct_prop_error + aKi * aErrSum + errDer*aKd
    #Remember some variables for next time
    aLastErr = error
    if (out_signal>2.7):
        out_signal = 2.65 
    if (out_signal<-2.75):
        out_signal = -2.65 
    direct_adj_signal = out_signal*turnRight
    #tb.set_control_inputs(0, out_signal) # set control input {lin-vel: 0, ang-vel: out_signal}
    return out_signal

def reachCoordinates_constantVel(input_x, input_y,constSpeed):
    #determination of direction
    #global relative_x, relative_y
    desired_angle_func = get_angle_to_next_coord(input_x, input_y)
    theta_output = reach_correct_angle_signal(desired_angle_func)
    tb.set_control_inputs(constSpeed, theta_output) # set control input {lin-vel: 0, ang-vel: out_signal}
    return

def reach_following_coordinates(coordinateList,SpeedUsed,sensetivityUsed):
    #infinnite loop of following
    # case 1 = normal movement
    # case 2 = turn in 1 place
    # case 3 = fast turn + slow down   
    global targetTReached
    while timeDif < 180:
        #update current list
        if (loopCounter == 1 or targetTReached == True):
            current_target = coordinateList[0]
            coordinateList.append(current_target)
            coordinateList.pop(0)
            targetting_angle = get_angle_to_next_coord(current_target[0], current_target[1])
            targetTReached = False

        if (abs(targetting_angle)< np.pi/4):
            #caseNum = 1
            reachCoordinates_constantVel(current_target[0], current_target[1], SpeedUsed)
        elif (abs(targetting_angle)< np.pi/2):
            #caseNum = 2
            reachCoordinates_constantVel(current_target[0], current_target[1], SpeedUsed/2)
        else:
            #caseNum = 3
            reach_correct_angle(targetting_angle)

        if (get_distance_to_coordinate(current_target)<sensetivityUsed):
            targetTReached = True
        
    return None

def setVelTunings(input_Kp, input_Ki, input_Kd):
    global vKp, vKi, vKd
    vKp = input_Kp
    vKi = input_Ki
    vKd = input_Kd
    return None

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
    return None       

while robotRunning:
    if loopCounter == 1:
        IMUList = []
        abscissaList = [] # for plotting graph
        ordinateList = [] # for plotting graph
        ordinateList2 = [] # for plotting graph
        
        
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

        vKp = 1.2
        vKi = 2.85
        vKd = 0.126

        aKp = 0.3
        aKi = 0
        aKd = 0

        dKp = 1
        dKi = 0
        dKd = 0

        vErrSum = 0
        vLastErr = 0

        aErrSum = 0
        aLstErr = 0

        dErrSum = 0
        dLastErr = 0
        
        theta = 0

        distance_travelled = 0
        angle_total =0
        forward_velocity = 0
        wall_y_coordinate = 1.75
        
        refTickLeft = dataList['left']
        refTickRight = dataList['right']
        
        #test1: do a 360
        #reach_correct_angle_total(360)
        #test2: move 2m ACTIVATE INTEGRALS AND DERIVATIVES
        #reach_correct_distance(2)
        
        #test3: go [1,1]
        #reachCoordinates_constantVel(1, 1, 0.05)

        #reach_correct_speed(0.05)
        #reach_correct_angle(np.pi*3/2)
        #reach_correct_distance(2)

        #MIMO test
        #reach_following_coordinates([[0,1],[2,1],[0,0]], 0.07, 0.05)

        #tb.set_control_inputs(0.1, 0.1) # set control input {lin-vel: 0.1, ang-vel:0} 
        
    returnedList = []
    returnedList = data_to_list(returnedList)
    dataList = returnedList[0]
    timeDif = returnedList[1]
    IMUList.append(tb.get_imu())

    leftTick = dataList['left']
    rightTick = dataList['right']
    
    forward_velocity = get_linear_velocity() 
    angular_velocity = get_angular_velocity() #* 57.29 # from radians to degrees
    update_distance_moved()
    update_angle_total()
    theta = get_current_theta()
    
    distance_to_wall = get_distance_to_wall() # distance according to the mathematical estimations. Based on the initial position and encoder ticks
    abscissaList.append(timeDif) # total time passed, since robot started
    ordinateList.append(distance_to_wall)
    
    nearestObject = get_nearest_object() # distance between the robot and nearest object detected. The scan is used
    ordinateList2.append(nearestObject)
    
    
    if loopCounter > 1:
        x_position = update_xposition()
        y_position = update_yposition()
    
    if loopCounter % 500 == 0:
        print("\nLinear velocity: ", str(round(forward_velocity, 5)))
        print("Angular velicity: ", str(round(angular_velocity, 5)))
        print("Angle: ", str(round(theta/2/np.pi*360, 5)))
        print("x position: ", str(round(x_position,5)))
        print("y position: ", str(round(y_position,5)))
        print("total distance travelled", str(round(distance_travelled,5)))
        print("total angular displacement", str(round(distance_travelled,5)))
            
    #previousTimeDif = timeDif
    
    if timeDif2 < 0.1:
        sleepTime = 0.1 - timeDif2
        time.sleep(timeDif2)
    
    loopCounter += 1
    
        
    #basic turn off    
    if timeDif > 10:
        robotRunning = False

    #turn off SISO#1
    #if angle_total>362:
        #robotRunning = False

    #turn off SISO#2
  #  if distance_travelled>2.03:
  #      robotRunning = False 
    #turn of SISO#3
    #if (np.mod(relative_x)<0.025 and np.mod(relative_y)<0.025):
      #  robotRunning = False    
        
# Saving the data plotted into a csv file
saveThis = []
saveThis.append(abscissaList)
saveThis.append(ordinateList)      
saveThis.append(ordinateList2)   
csvName = "Data plot"    
data_to_csv(saveThis, csvName)

# Plotting the data
plot_this(abscissaList, ordinateList)
plot_this(abscissaList, ordinateList2)

tb.stop()
        

    
