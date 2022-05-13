from methods import p_controllers
from methods import pid_controllers
from methods import getters

def reach_forward_speed(inputForwVel,inputForwardVel):
    print (p_controllers.p_controller_speed_signal(inputForwVel,inputForwardVel))
    tb.set_control_inputs(p_controllers.p_controller_speed_signal(inputForwVel,inputForwardVel), 0) # set control input {lin-vel: 0, ang-vel: out_signal}
    #tb.set_control_inputs(pid_controllers.p_controller_speed_signalExp(inputForwVel,inputForwardVel), 0) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_correct_angle_0_forward_vel(set_angle,inputTheta):
    tb.set_control_inputs(0, p_controllers.p_controller_angle_signal(set_angle,inputTheta)) # set control input {lin-vel: 0, ang-vel: out_signal}
    #tb.set_control_inputs(0, p_controllers.pid_controller_angle_signalExp(set_angle,inputTheta)) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_theta_travelled_0_forward_vel(total_theta_wanted,inputTotalTheta):
    tb.set_control_inputs(p_controllers.p_controller_theta_travelled_angle_velocity_signal(total_theta_wanted,inputTotalTheta), 0) # set control input {lin-vel: 0, ang-vel: out_signal}
    #tb.set_control_inputs(pid_controllers.p_controller_theta_travelled_angle_velocity_signalExp(total_theta_wanted,inputTotalTheta), 0) # set control input {lin-vel: 0, ang-vel: out_signal}
    return None

def reach_distance_0_angular_vel(inputDistance,inputCurrentDistance):
    tb.set_control_inputs(0.1, p_controllers.p_controller_distance_travelled_forward_velocity_signal(inputDistance,inputCurrentDistance)) # set control input {lin-vel: 0.1, ang-vel:0}
    return None

def reach_coordinates_constantVelocity(inputCoordList,constSpeed):
    desired_angle_func = getters.get_perfect_angle_to_next_coord_from_current_position(inputCoordList)
    theta_output = p_controllers.p_controller_angle_signal(desired_angle_func,inputTheta)
    tb.set_control_inputs(constSpeed, theta_output) # set control input {lin-vel: 0, ang-vel: out_signal}
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
            listOfSeqCoord.append(getters.get_coordinates_behind_point_angle(inputCoordList, distanceBehind_itterative))
    if (getrters.get_distance_to_coordinate(listOfSeqCoord[0])<sensetivityDist): #check if distance is close enough
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
    
    if (getters.get_distance_to_coordinate(infCoordList[0])<sensetivityUsed):
        infCoordList.append(currentCoordTargetMIMO)
        infCoordList.pop(0)
        aErrorList = np.zeros(4)
        aTimeDifferences = np.zeros(4)

    currentCoordTargetMIMO = infCoordList[0]
    targetting_angle = getters.get_perfect_angle_to_next_coord_from_current_position(currentCoordTargetMIMO)

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
