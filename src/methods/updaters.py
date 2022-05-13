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
    if timeFromStart != 0:
        velocityLeftWheel = get_tick_value_in_rad(leftTickCurrent - refTickLeft) * 0.066 * 0.5/timeChange_2lastUpdates
        velocityRightWheel = get_tick_value_in_rad(rightTickCurrent - refTickRight) * 0.066 * 0.5/timeChange_2lastUpdates
        linear_velCalc = (velocityRightWheel + velocityLeftWheel)/2 
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
    IMUList.append(tb.get_imu())
    abscissaList.append(timeFromStart)
    ordinateList.append(distance_to_wall)
    nearestObject = get_nearest_object() # distance between the robot and nearest object detected. The scan is used
    ordinateList2.append(nearestObject)
    return None
