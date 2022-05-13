
import methods.data.constants
def p_controller_speed_signal(set_LinVel):
    import testfile
    prop_error = set_LinVel - testfile.forward_velocity
    out_signal = methods.data.constants.vKp * prop_error 
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
