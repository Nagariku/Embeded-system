# -*- coding: utf-8 -*-
"""
Created on Wed May 11 2022
@author: nagariku and Cory
"""

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


listOfSeqCoord = []
def setDistanceTunings(input_Kp, input_Ki, input_Kd):
    global dKp, dKi, dKd
    dKp = input_Kp
    dKi = input_Ki
    dKd = input_Kd
    return None       


    #listOfSeqCoord = [[0,0],[0,2],[3,1]]
    #current_target = listOfSeqCoord[2][0]
    #print(current_target)
    #3
    global thetaTargetAngle, targetTReached, listOfSeqCoord,distSplit,numAcc,finalReached,n,current_target
    if (globalLoopCounter==0):
        n=0
        targetTReached = False
        finalReached = False
        thetaTargetAngle = inputCoordList [2] 
        numAcc = 3 #number of accuracy, higher = better, too high = oscilations
        distSplit = dist/numAcc
        for i in range(0,numAcc+1,1):
            distanceBeh = dist - i*distSplit
            listOfSeqCoord.append(get_coordinates_behind_point_angle(inputCoordList, distanceBeh,thetaTargetAngle))
    if (len(listOfSeqCoord)==0):
        finalReached = True
    if (finalReached==False) :
        if (globalLoopCounter ==0 or targetTReached == False):
            if (globalLoopCounter==0):
                current_target = listOfSeqCoord[0]         
                
            if (len(listOfSeqCoord)==0):
                finalReached = True
        if (get_distance_to_coordinate(current_target)<0.1):
            targetTReached = True   
            current_target = listOfSeqCoord[0]
            listOfSeqCoord.pop(0)
        else:
            targetTReached = False
        reach_coordinates_constantVelocity(current_target, constVel)
    return None

def reach_following_coordinates(coordinateList,SpeedUsed,sensetivityUsed)
###reach certain coordinates
    #infinite loop of following
    # case 1 = normal movement
    # case 2 = turn in 1 place
    # case 3 = fast turn + slow down 
    targetTReached = False
    while timeFromStart < 180:
        #update current list
        if (globalLoopCounter ==0 or targetTReached == True):
            current_target = coordinateList[0]
            coordinateList.append(current_target)
            coordinateList.pop(0)
            targetting_angle = get_perfect_angle_to_next_coord_from_current_position([current_target[0], current_target[1]])
            targetTReached = False

        if (abs(targetting_angle)< np.pi/4):
            #caseNum = 1
            reach_coordinates_constantVelocity(current_target, SpeedUsed)
        elif (abs(targetting_angle)< np.pi/2):
            #caseNum = 2
            reach_coordinates_constantVelocity(current_target, SpeedUsed/2)
        else:
            #caseNum = 3
             reach_correct_angle_0_forward_vel(targetting_angle)

        if (get_distance_to_coordinate(current_target)<sensetivityUsed):
            targetTReached = True

            
#turn off SISO#1
if angle_total>np.pi*2.1:
    robotRunning = False

#turn off SISO#2
#if distance_travelled>2.03:
    # robotRunning = False 
#turn of SISO#3
#relX = np.abs(1-current_x)
#relY = np.abs(1-current_y)
#if (relX<0.05 and relY<0.05):
#    robotRunning = False    