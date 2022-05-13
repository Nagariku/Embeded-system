def import_variables():
    ###Initialising variables
    ##aka don't touch these or program won't work

    #Turtlebot instance
    outTb = Turtlebot() 

    #booleans
    outRobotRunning = True
    outTimeTickUpdate_bool = False

    #lists
    outListZeros =  [0,0]
    #  timeFromStartArray = DeadReckon_List_theta = DeadReckon_List_vel = current_target = [0,0]
    #Graph plotting CORY
    outListsEmpty=  [] # for plotting graph
    #IMUList = abscissaList = ordinateList = ordinateList2 = []
    outGlobalLoopCounter = 1

    #floats     
    unchangedZero  = 0
    # current_x = current_y =  distance_travelled= angle_total =   = global_velocity_signal=   =0
    return outTb,outRobotRunning,outTimeTickUpdate_bool,outListZeros, outListsEmpty, outGlobalLoopCounter, unchangedZero
