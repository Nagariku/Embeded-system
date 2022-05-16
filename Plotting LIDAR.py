# -*- coding: utf-8 -*-
"""
Created on Tue May 10 11:16:17 2022

@author: admin

Questions:
    -IMU time stamp, is sec and nano the same value, but different resolution,
    or should they be summed
    -IMU angle is absolute
    -Allowed to just use the IMU angle
    -In while loop, to spare CPU usage use time.sleep() Avoid overhead
"""
import pandas as pd
import csv
import numpy as np
import matplotlib.pyplot as plt
from rolab_tb.turtlebot import Turtlebot
import time
import json
from scipy.interpolate import make_interp_spline

def data_to_csv(listToSave, csvName):
    """
    Works: but watch out with csv file name when rrunning the cod emultiple times

    Parameters
    ----------
    listToSave : TYPE
        DESCRIPTION.
    csvName : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    name = csvName + ".csv" #"Saved data angular"+ ".csv" # change name according to recording type
    dict = {name: listToSave}
    df = pd.DataFrame(dict)
    df.to_csv(name)
    np.savetxt(name, listToSave, delimiter =", ", fmt ='% s')

def plot_this(abscissaList, ordinateList):
    """
    Parameters
    ----------
    abscissaList : TYPE
        DESCRIPTION.
    ordinateList : TYPE
        DESCRIPTION.

    Returns
    -------
    None.
    
    Works

    """
    abscissaList = np.asarray(abscissaList)
    ordinateList = np.asarray(ordinateList)
    B_spline_coeff = make_interp_spline(abscissaList, ordinateList)
    X_Final = np.linspace(abscissaList.min(), abscissaList.max(),75) #choose the resolution 
    Y_Final = B_spline_coeff(X_Final)

    plt.plot(X_Final, Y_Final)
    plt.ylabel('Angular speed (rad/s)') #set the label for y axis
    plt.xlabel('Time (s)') #set the label for x-axis
    plt.title("Angular mvt") #set the title of the graph
    plt.grid()
    plt.show() #display the graph

def get_nearest_object():
    """
    Works
    
    Returns
    -------
    closest_ting : TYPE
        DESCRIPTION.

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

# IMU1 = {'header': {'stamp': {'sec': 1650618710, 'nanosec': 199955325}, 'frame_id': 'imu_link'}, 'orientation': {'x': 0.013579659163951874, 'y': -0.0014849099097773433, 'z': -0.0695350393652916, 'w': 0.9974817037582397}, 'orientation_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular_velocity_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'linear_acceleration': {'x': 0.14903905987739563, 'y': 0.29388824105262756, 'z': 10.141838073730469}, 'linear_acceleration_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
# IMU2 = {'header': {'stamp': {'sec': 1650618710, 'nanosec': 449978189}, 'frame_id': 'imu_link'}, 'orientation': {'x': 0.01264654379338026, 'y': -0.010978278703987598, 'z': -0.0628562644124031, 'w': 0.9978777766227722}, 'orientation_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'angular_velocity': {'x': -0.009577799588441849, 'y': -0.07981500029563904, 'z': 0.07449399679899216}, 'angular_velocity_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'linear_acceleration': {'x': 0.8122329115867615, 'y': 0.13706804811954498, 'z': 10.333972930908203}, 'linear_acceleration_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
# IMU3 = {'header': {'stamp': {'sec': 1650618710, 'nanosec': 649948398}, 'frame_id': 'imu_link'}, 'orientation': {'x': 0.015011219307780266, 'y': -0.019932419061660767, 'z': -0.06092480197548866, 'w': 0.9978261590003967}, 'orientation_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'angular_velocity': {'x': 0.017027199268341064, 'y': 0.06065940111875534, 'z': 0.02447660081088543}, 'angular_velocity_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'linear_acceleration': {'x': 1.357512354850769, 'y': 0.8301894068717957, 'z': 11.443086624145508}, 'linear_acceleration_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
# IMU4 = {'header': {'stamp': {'sec': 1650618711, 'nanosec': 99998293}, 'frame_id': 'imu_link'}, 'orientation': {'x': 0.013251585885882378, 'y': -0.020493699237704277, 'z': -0.05424445495009422, 'w': 0.9982250928878784}, 'orientation_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'angular_velocity': {'x': -0.06385199725627899, 'y': 0.015962999314069748, 'z': 0.08087919652462006}, 'angular_velocity_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'linear_acceleration': {'x': 0.9876081943511963, 'y': 1.8471266031265259, 'z': 8.938153266906738}, 'linear_acceleration_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
# IMU5 = {'header': {'stamp': {'sec': 1650618714, 'nanosec': 49960844}, 'frame_id': 'imu_link'}, 'orientation': {'x': 0.01361268013715744, 'y': -0.014897918328642845, 'z': -0.013262826018035412, 'w': 0.9997040629386902}, 'orientation_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'angular_velocity': {'x': 0.03724699839949608, 'y': -0.02447660081088543, 'z': -0.04256799817085266}, 'angular_velocity_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'linear_acceleration': {'x': -0.7314286231994629, 'y': 0.050876785069704056, 'z': 10.175955772399902}, 'linear_acceleration_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}


# IMUList = []
# IMUList.append(IMU1)
# IMUList.append(IMU2)
# IMUList.append(IMU3)
# IMUList.append(IMU4)
# IMUList.append(IMU5)


# print(IMU1['header'])
# print(IMU1['orientation']['y'])

abscissaList = []
ordinateList = []
# for i in IMUList:
#     theTime = i['header']['stamp']['sec'] + i['header']['stamp']['nanosec'] * 10**(-9)
#     abscissaList.append(theTime)
    
#     # abscissaList.append(i['orientation']['z'])
#     ordinateList.append( i['orientation']['w'])
#     # print("\nX is: ",i['orientation']['x'])
#     # print("Y is: ", i['orientation']['y'])
    
# print(abscissaList)
# print(ordinateList)


myCounter = 0
letsgo = True
tb = Turtlebot() 

try:
    while letsgo:
        if myCounter == 0:
            time1 = time.time()
            # tb.set_control_inputs(0.2, 0.1) # set control input {lin-vel: 0.2, ang-vel:0.1}  
        
        time2 = time.time()
        timeDif = time2 - time1
        proximity = get_nearest_object()
        
        abscissaList.append(timeDif)
        ordinateList.append(proximity)
        
        print(proximity)
        
        time.sleep(1)
        myCounter += 1
        
except KeyboardInterrupt:
    pass

tb.stop() 

plot_this(abscissaList, ordinateList)
# listToSave = []
# listToSave.append(abscissaList)
# listToSave.append(ordinateList)

# csvName = "save the thing"    
# data_to_csv(listToSave, csvName)

