# -*- coding: utf-8 -*-
"""
Created on Tue May  3 14:28:13 2022

@author: nagariku and Cory
"""

####Imports
# from distutils.log import error
import time
from rolab_tb.turtlebot import Turtlebot
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import pandas as pd
import json

#coordinates are [x,y] or [x,y,theta]
#x and y are in meters
#theta in radians from 0 to 2pi in anticlockwise direction, if overflowed it resets to 0
#tb.set_control_inputs(0.1, 0.1) # set control input {lin-vel: 0.1, ang-vel:0}  aka base of controls

#test
#p_controller_theta_travelled_angle_velocity_signal with higher speeds

###Getters
def get_data_from_sensors():
    global timeatStart
    returnedList =[]
    timeatStart = time.time()
    returnedList = get_data_to_list(returnedList)
    dataList = returnedList[0]
    outTimeCalc = returnedList[1] 
    outTickLeft = dataList['left']
    outTickRight = dataList['right']
    return outTickLeft, outTickRight , outTimeCalc

def get_data_to_list(listToSave):
    '''
    Calculates current angular velocity from 2 global tick values - from top of robot clockwise is negative, anticlockwise is possitive

            Parameters:
                    leftTickCurrent (float) current tick of left encoder
                    rightTickCurrent (float) current tick of right encoder
                    timeChange_2lastUpdates (float) global time change of last 2 time readings in seconds
            Returns:
                    angular_velCalc (float) Current angular velocity of robot   
    '''    

    global timeChange_2lastUpdates, timeTickUpdate_bool, timeFromStart

    timeCurrent = time.time()   
    timeFromStart = timeCurrent - timeatStart 
  
    timeFromStartArray.append(timeFromStart)
    timeFromStartArray.pop(0)
    timeChange_2lastUpdates = timeFromStartArray[1]-timeFromStartArray[0]

    if (timeChange_2lastUpdates!=0):
        print(" \nCONDITION MET")
        timeTickUpdate_bool = True
        DeadReckon_List_vel.append(forward_velocity)
        DeadReckon_List_theta.append(get_current_theta())
        DeadReckon_List_vel.pop(0)
        DeadReckon_List_theta.pop(0)
    else:    
        timeTickUpdate_bool = False

    listToSave.append(tb.get_encoder_ticks())
    listToSave.append(timeFromStart)
    
    return listToSave

def get_tick_value_in_rad(inputEncoderTick):
    '''
    Converts the encoders ticks into radians using x*input where input is 001533981f (0.087890625[deg] * 3.14159265359 / 180 )

            Parameters:
                    tickInDegrees (int): An endocder integer in

            Returns:
                    tickInRad (float): Encoder input tick converted to radians

            Notes:
                    2^12 = 4096 ticks/revolution
    '''
    tickInRad = inputEncoderTick * 0.001533981 
    return tickInRad

def get_current_theta():
    right_tick_relative = rightTickCurrent - refTickRight
    left_tick_relative = leftTickCurrent - refTickLeft
    differenceInTicks = right_tick_relative - left_tick_relative
    if differenceInTicks > 0:
        remaindery = differenceInTicks % 19859
        thetaCalc = remaindery*0.0181274
        thetaCalc = thetaCalc/360*2*np.pi #degrees to radians
    elif differenceInTicks < 0:
        remaindery = (-differenceInTicks) % (-19859)
        thetaCalc = remaindery*(-0.0181274) 
        thetaCalc = thetaCalc/360*2*np.pi #degrees to radians
    else:
        thetaCalc = 0
    return thetaCalc

def get_deadreckon_v_and_angle_averages():
    #correction to prevent negative angle fuckery
    averageSpeed = (DeadReckon_List_vel[1]+DeadReckon_List_vel[0])/2
    averageTheta = (DeadReckon_List_theta[1] + DeadReckon_List_theta[0])/2
    if ((max(DeadReckon_List_theta)-min(DeadReckon_List_theta))>1):
        biggerTheta = -2*np.pi+max(DeadReckon_List_theta)
        smallerTheta = min(DeadReckon_List_theta)
        averageTheta = (biggerTheta + smallerTheta)/2
        if averageTheta<0:
            return averageSpeed, 2*np.pi+averageTheta,
    return averageSpeed, averageTheta

def get_distance_to_coordinate(listCoordsInput):
    delta_X=current_x - listCoordsInput[0]
    delta_Y= current_y - listCoordsInput[1]
    totalDistanceToCoord = np.sqrt(delta_Y**2+delta_X**2)
    return totalDistanceToCoord

def get_distance_to_wall():
    return wall_y_coordinate-current_y

def get_perfect_angle_to_next_coord_from_current_position(listCoordsInput):
    relative_x = listCoordsInput[0] - current_x
    relative_y = listCoordsInput[1] - current_y
    #checked manually
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

def get_coordinates_behind_point_angle(inputCoordListAngle,distanceBehind):
    out_coord_x = inputCoordListAngle[0] + distanceBehind * np.cos(inputCoordListAngle[2])
    out_coord_y = inputCoordListAngle[1] + distanceBehind * np.sin(inputCoordListAngle[2])
    return out_coord_x, out_coord_y

def get_nearest_object():
    """
    Works
    Determines using the LIDAR where the nearest object is
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

###Updaters
def update_current_linear_velocity():
    '''
    Calculates current forward velocity from 2 global tick values

            Parameters:
                    leftTickCurrent (float) current tick of left encoder
                    rightTickCurrent (float) current tick of right encoder
                    timeChange_2lastUpdates (float) global time change of last 2 time readings in seconds
            Returns:
                    linear_velCalc (float)Current forward velocity of robot   
    '''
    #update when time tick
    if timeTickUpdate_bool == True:
        velocityLeftWheel = get_tick_value_in_rad(leftTickCurrent - refTickLeft) * 0.0000066 * 0.5/timeChange_2lastUpdates #0.066
        velocityRightWheel = get_tick_value_in_rad(rightTickCurrent - refTickRight) * 0.0000066 * 0.5/timeChange_2lastUpdates
        linear_velCalc = (velocityRightWheel + velocityLeftWheel)/2
    else:
        linear_velCalc =  0
    return linear_velCalc

def update_current_angular_velocity():
    '''
    Calculates current angular velocity from 2 global tick values - from top of robot clockwise is negative, anticlockwise is possitive

            Parameters:
                    leftTickCurrent (float) current tick of left encoder
                    rightTickCurrent (float) current tick of right encoder
                    timeChange_2lastUpdates (float) global time change of last 2 time readings in seconds
            Returns:
                    angular_velCalc (float) Current angular velocity of robot   
    '''    
    if timeFromStart != 0:
        velocityLeftWheel = get_tick_value_in_rad(leftTickCurrent - refTickLeft) * 0.066 * 0.5/timeChange_2lastUpdates
        velocityRightWheel = get_tick_value_in_rad(rightTickCurrent - refTickRight) * 0.066 * 0.5/timeChange_2lastUpdates
        angular_velCalc = (velocityRightWheel-velocityLeftWheel)/0.16 
    else:
        angular_velCalc = 0
    return angular_velCalc

def update_distance_travelled():
    distance_travelledCalc = distance_travelled + np.sqrt(change_x**2 + change_y**2) # Euclidian distance assumes the distance traveled is the shortest one (no curves, turns etc)
    return distance_travelledCalc

def update_theta_travelled():
    if ((max(DeadReckon_List_theta)-min(DeadReckon_List_theta))>1): # sensetivity
        biggerTheta = -2*np.pi+max(DeadReckon_List_theta)
        smallerTheta = min(DeadReckon_List_theta)
        angle_totalCalc = angle_total + smallerTheta - biggerTheta
    else:
        angle_totalCalc = angle_total + (max(DeadReckon_List_theta)-min(DeadReckon_List_theta)) # Euclidian distance assumes the distance traveled is the shortest one (no curves, turns etc)
    return angle_totalCalc

def update_xposition():
    vAverageTick, thetaAverageTick = get_deadreckon_v_and_angle_averages()
    outChange_x = vAverageTick*np.cos(thetaAverageTick)*(timeChange_2lastUpdates) # error appearing when speed is not constant
    current_xCalc = current_x + outChange_x
    return current_xCalc, outChange_x

def update_yposition():
    vAverageTick, thetaAverageTick = get_deadreckon_v_and_angle_averages()
    outChange_y = vAverageTick*np.sin(thetaAverageTick)*(timeChange_2lastUpdates) # error appearing when speed is not constant
    current_yCalc = current_y + outChange_y
    return current_yCalc, outChange_y

def update_graph_data():
    # IMUList.append(tb.get_imu())
    abscissaList.append(timeFromStart)
    ordinateList.append(distance_to_wall)
    nearestObject = get_nearest_object() # distance between the robot and nearest object detected. The scan is used
    ordinateList2.append(nearestObject)
    return None

###P controllers
def p_controller_speed_signal(set_LinVel):
    prop_error = set_LinVel - forward_velocity
    out_signal = vKp * prop_error 
    final_signal = set_LinVel+out_signal
    if (final_signal >0.22):
        final_signal  = 0.215
    return final_signal

def p_controller_angle_signal(set_angle):
    prop_error = set_angle - theta

    if (prop_error ==0 or np.pi):
        turnRight = 1
        #cause why not, no need to make it an RNG
    elif (set_angle>np.pi):
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
        correct_prop_error = np.pi*2-prop_error
    elif (prop_error < -np.pi):
        correct_prop_error =  2*np.pi+prop_error
    elif (prop_error < 0):
        correct_prop_error =  -prop_error
    else:
        correct_prop_error = prop_error

    out_signal = aKp * correct_prop_error 
    if (out_signal>2.7):
        out_signal = 2.65 
    elif (out_signal<-2.75):
        out_signal = -2.65 
    direct_adj_signal = out_signal*turnRight
    return direct_adj_signal

def p_controller_distance_travelled_forward_velocity_signal(set_distance):
    prop_error = set_distance - distance_travelled
    out_signal = dKp * prop_error #+ dKi * dErrSum #+ errDer*dKd
    if (out_signal>0.22):
        out_signal = 0.215
    return out_signal

def p_controller_theta_travelled_angle_velocity_signal(set_angle_total):
    prop_error = set_angle_total - angle_total
    out_signal = aDKp * prop_error 
    #try higher
    if (out_signal>0.5):
        out_signal = 0.4 
    if (out_signal<-0.5):
        out_signal = -0.4 
    return out_signal

###P controllers - experimental
def p_controller_speed_signalExp(set_LinVel):
    #initialise varaiables
    global global_velocity_signal

    #Calculatre error
    prop_error = set_LinVel - forward_velocity

    #calculate variables
    vLastErr = aErrorList[len(aErrorList)-1]
    errorDerivative = (prop_error - vLastErr)/timeChange_2lastUpdates
    vErrAverage = sum(vErrorList)/len(vErrorList)
    vErrSum = vErrAverage * sum(vTimeDifferences)
    global_velocity_signal = vKp * prop_error + vKi * vErrSum +  vKd* errorDerivative

    #set up variables for next time
    vErrorList.pop(0)
    vErrorList.append(prop_error)
    vTimeDifferences.pop(0)
    vTimeDifferences.append(timeChange_2lastUpdates)

    #outputting the stuff
    if (global_velocity_signal >0.22):
        global_velocity_signal  = 0.215
    elif (global_velocity_signal <-0.22):
        global_velocity_signal  = -0.215
    return global_velocity_signal

def p_controller_angle_signalExp(set_angle):
    #calculate error
    prop_error = set_angle - theta

    if (prop_error ==0 or np.pi):
        turnRight = 1
        #cause why not, no need to make it an RNG
    elif (set_angle>np.pi):
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
        correct_prop_error = np.pi*2-prop_error
    elif (prop_error < -np.pi):
        correct_prop_error =  2*np.pi+prop_error
    elif (prop_error < 0):
        correct_prop_error =  -prop_error
    else:
        correct_prop_error = prop_error

    #reset 2 lists on reach

    #calculate signal
    aLastErr = aErrorList[len(aErrorList)-1]
    errorDerivative = (correct_prop_error- aLastErr)/timeChange_2lastUpdates
    aErrAverage = sum(aErrorList)/len(aErrorList)
    aErrSum = aErrAverage * sum(aTimeDifferences)
    
    out_signal = aKp * correct_prop_error + aKi*aErrSum + aKd *errorDerivative  

    #variable calculation for later
    aErrorList.pop(0)
    aErrorList.append(correct_prop_error)
    aTimeDifferences.pop(0)
    aTimeDifferences.append(timeChange_2lastUpdates)

    #outputting stuff
    if (out_signal>2.7):
        out_signal = 2.65 
    elif (out_signal<-2.75):
        out_signal = -2.65 
    direct_adj_signal = out_signal*turnRight
    return direct_adj_signal

###Calculations and actuators
def reach_forward_speed(inputForwVel):
    #tb.set_control_inputs(p_controller_speed_signal(inputForwVel), 0) # set control input {lin-vel: 0, ang-vel: out_signal}
    tb.set_control_inputs(p_controller_speed_signalExp(inputForwVel), 0) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_correct_angle_0_forward_vel(set_angle):
    tb.set_control_inputs(0, p_controller_angle_signal(set_angle)) # set control input {lin-vel: 0, ang-vel: out_signal}
    #tb.set_control_inputs(0, p_controller_angle_signalExp(set_angle)) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_theta_travelled_0_forward_vel(total_theta_wanted):
    tb.set_control_inputs(p_controller_theta_travelled_angle_velocity_signal(total_theta_wanted), 0) 
    return None

def reach_distance_0_angular_vel(inputDistance):
    tb.set_control_inputs(0.1, p_controller_distance_travelled_forward_velocity_signal(inputDistance)) # set control input {lin-vel: 0.1, ang-vel:0}
    return None

def reach_coordinates_constantVelocity(inputCoordList,constSpeed):
    desired_angle_func = get_perfect_angle_to_next_coord_from_current_position(inputCoordList)
    theta_output = p_controller_angle_signal(desired_angle_func)
    #theta_output = p_controller_angle_signalExp(desired_angle_func)
    velocity_output = p_controller_speed_signal(constSpeed)
    velocity_output = p_controller_speed_signalExp(constSpeed)
    tb.set_control_inputs(velocity_output, theta_output) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_coordinates_and_angle(inputCoordList, constVel, distanceBehindPoint, inputNumPoints):
    global targetReachedFinalSISO,listOfSeqCoords, currentCoordTargetSISO, aErrorList, aTimeDifferences
    if (globalLoopCounter == 1): #setting up for future
        #desired theta is inputCoordList[2]
        targetReachedFinalSISO = False
        distSplit = distanceBehindPoint/inputNumPoints
        listOfSeqCoords=[]
        for i in range(0,inputNumPoints+1,1):
            distanceBehind_itterative = distanceBehindPoint - i*distSplit
            listOfSeqCoords.append(get_coordinates_behind_point_angle(inputCoordList, distanceBehind_itterative))
    if (get_distance_to_coordinate(listOfSeqCoords[0])<sensetivityDist): #check if distance is close enough
        listOfSeqCoords.pop(0)
        if (len(listOfSeqCoords)>0): #check if there is next coordinator
            currentCoordTargetSISO = listOfSeqCoords[0]
            aErrorList = np.zeros(4)
            aTimeDifferences = np.zeros(4)
        else: #if not, target is reached
            targetReachedFinalSISO = True
            return None
    else:
            currentCoordTargetSISO = listOfSeqCoords[0]

    reach_coordinates_constantVelocity(listOfSeqCoords[0], constVel)
    return currentCoordTargetSISO

def reach_following_coordinates(inCoordinateList,inSpeedUsed,sensetivityUsed):
    #infinite loop of following
    ###calculator part
    global  currentCoordTargetMIMO, infCoordList, aErrorList, aTimeDifferences #targetReachedMIMO
    

    if (globalLoopCounter == 1):
        infCoordList = inCoordinateList
        currentCoordTargetMIMO = infCoordList[0]
        if (len(inCoordinateList)<2):
            print ("Insufficient coordinates")
        return None
    
    if (get_distance_to_coordinate(infCoordList[0])<sensetivityUsed):
        infCoordList.append(currentCoordTargetMIMO)
        infCoordList.pop(0)
        aErrorList = np.zeros(4)
        aTimeDifferences = np.zeros(4)

    currentCoordTargetMIMO = infCoordList[0]
    targetting_angle = get_perfect_angle_to_next_coord_from_current_position(currentCoordTargetMIMO)

    ###Actuator part
    if ((targetting_angle<(np.pi/3)) or (targetting_angle>5/3)):
        #caseNum = 1 normal movement
        reach_coordinates_constantVelocity(currentCoordTargetMIMO, inSpeedUsed)
    elif ((targetting_angle<(2*np.pi/3)) or (targetting_angle>4/3)):
        #caseNum = 2 fast turn + slow down 
        reach_coordinates_constantVelocity(currentCoordTargetMIMO, inSpeedUsed/2)
    else:
        #caseNum = 3 turn in 1 place
        reach_correct_angle_0_forward_vel(targetting_angle)
    return currentCoordTargetMIMO

###Graph plotting
def ouput_plot(abscissaList, ordinateList, aLegend):
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
    plt.legend(aLegend)
    plt.show() #display the graph

def save_to_csv(listToSave, csvName):
    """
    Works: but watch out with csv file name when running the code multiple times
    """
    name = csvName + ".csv" #"Saved data angular"+ ".csv" # change name according to recording type
    dict = {name: listToSave}
    df = pd.DataFrame(dict)
    df.to_csv(name)
    np.savetxt(name, listToSave, delimiter =", ", fmt ='% s')

def show_output_results():
    # Saves data to csv
    saveThis = []
    saveThis.append(abscissaList)
    saveThis.append(ordinateList)      
    saveThis.append(ordinateList2)   
    csvName = "Data plot"    
    save_to_csv(saveThis, csvName)

    # Plotting the data
    ouput_plot(abscissaList, ordinateList, legend1)
    ouput_plot(abscissaList, ordinateList2, legend2)
    return None

######################################
###Initialising variables
##aka don't touch these or program won't work

#Turtlebot instance
tb = Turtlebot() 

#booleans
robotRunning = True
timeTickUpdate_bool = False

#lists
timeFromStartArray = [0,0]
DeadReckon_List_theta = [0,0]
DeadReckon_List_vel = [0,0]
current_target=[0,0]

#Graph plotting CORY
# IMUList = []
legend1 = ["From Ticks"]
legend2 = ["From LIDAR"]
abscissaList = [] # for plotting graph, the time since start
ordinateList = [] # for plotting graph
ordinateList2 = [] # for plotting graph

#integers
globalLoopCounter = 1
refTickLeft = 0 #was None before
refTickRight = 0 #was None before

#floats     
current_x = 0
current_y = 0
change_x=0
change_y=0
theta = 0
distance_travelled = 0
angle_total = 0
forward_velocity = 0
angular_velocity = 0
global_velocity_signal = 0


###################################
###Editable variables
##aka edit them to change behaviour

sensetivityDist = 0.05

#PID controlls
#velocity controlls
vKp = 0.75
vKi = 0
vKd = 0.
vErrorList = [0,0,0,0] #each zero = 0.045sec
vTimeDifferences = [0,0,0,0] #each zero = 0.045sec

#angular controls
aKp = 0.6 # 0.35
aKi = 2.85
aKd = 0.126
aErrorList = [0,0,0,0] #each zero = 0.045sec
aTimeDifferences = [0,0,0,0] #each zero = 0.045sec

#distance travelled controls
dKp = 0.1
dKi = 0
dKd = 0.126

#total theta travelled controls        
aDKp = 0.3
aDKi = 0
aDKd = 0

#wall cooodinate axis
wall_y_coordinate = 1.75

##SISO inputs
#SISO 1.1
SISO_in_1_1 = 2*np.pi #360 degree turn

#SISO 1.2
SISO_in_1_2 = 2 # 2 meters

#SISO 1.3
SISO_in_1_3_list = [1,1]
SISO_in_1_3_velocity = 0.05

#SISO 1.4
SISO_in_1_4_list = [-1,-1, 3/2*np.pi]
SISO_in_1_4_velocity = 0.05 #m/s
SISO_in_1_4_distance = 1 # 1 meters
SISO_in_1_4_numPoints = 2

#MIMO 1.1
MIMO_in_1_1_list =[[0,1],[2,1],[0,0]]
MIMO_in_1_1_sensetivity = 0.05 #m
MIMO_in_1_1_velocity = 0.05 #m/s

######################
### Main program

# get data start 
refTickLeft, refTickRight , timeFromStart = get_data_from_sensors()

###loop
while robotRunning:
    ###get data
    leftTickCurrent, rightTickCurrent, timeFromStart = get_data_from_sensors()

    #code only changes when tick happens
    if (timeTickUpdate_bool==True):
        #get updates
        forward_velocity = update_current_linear_velocity() 
        angular_velocity = update_current_angular_velocity() #* 57.29 # from radians to degrees
        distance_travelled = update_distance_travelled()
        angle_total = update_theta_travelled()
        theta = get_current_theta()
        distance_to_wall= get_distance_to_wall()
        current_x, change_x = update_xposition()
        current_y, change_y = update_yposition()
        
        ###Graph stuff
        update_graph_data()

        #current function being completed
        reach_forward_speed(0.05)
        #PASTE HERE
    
    
    if globalLoopCounter % 500 == 0:
        print("\nLinear velocity: ", str(round(forward_velocity, 5)))
        print("Angular velicity: ", str(round(angular_velocity, 5)))
        print("Angle: ", str(round(theta/2/np.pi*360, 5)))
        print("x position: ", str(round(current_x,5)))
        print("y position: ", str(round(current_y,5)))
        print("current target: ", current_target) # infCoordList
        print("distance to target", get_distance_to_coordinate(current_target))
        print("total distance travelled", str(round(distance_travelled,5)))
        print("total angular displacement", str(round(angle_total,5)))
    
    #basic turn off    
    if timeFromStart > 20:
        robotRunning = False

    #PASTE TURN OFFS HERE    


    globalLoopCounter += 1
    
tb.stop()
        
###outputs when out of loop, can be commented out
show_output_results()