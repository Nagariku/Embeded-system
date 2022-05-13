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
    global_velocity_signal = vKp * prop_error + vKi * vErrAverage +  vKd* errorDerivative

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
