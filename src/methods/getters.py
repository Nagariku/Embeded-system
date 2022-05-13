def get_data_from_sensors():
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

    global timeChange_2lastUpdates, timeTickUpdate_bool

    timeCurrent = time.time()   
    timeFromStart = timeCurrent - timeatStart 
  
    timeFromStartArray.append(timeFromStart)
    timeFromStartArray.pop(0)
    timeChange_2lastUpdates = timeFromStartArray[1]-timeFromStartArray[0]

    if (timeChange_2lastUpdates!=0):
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
    if ((max(averageTheta)-min(averageTheta))>1):
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
