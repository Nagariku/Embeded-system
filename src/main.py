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

#files
from methods import actuators, getters, graph_plotting, p_controllers, pid_controllers, updaters
from data import constants

#coordinates are [x,y] or [x,y,theta]
#x and y are in meters
#theta in radians from 0 to 2pi in anticlockwise direction, if overflowed it resets to 0
#tb.set_control_inputs(0.1, 0.1) # set control input {lin-vel: 0.1, ang-vel:0}  aka base of controls

tb = Turtlebot()
robotRunning = True
timeTickUpdate_bool = False
timeFromStartArray = DeadReckon_List_theta = DeadReckon_List_vel = current_target = [0,0] 
IMUList = abscissaList = ordinateList = ordinateList2 = []
globalLoopCounter = 1
current_x = current_y =  distance_travelled= angle_total =  global_velocity_signal= 0


if __name__ == "__main__": 
    # get data start 
    refTickLeft, refTickRight , timeFromStart = getters.get_data_from_sensors()

    ###loop
    while robotRunning:
        ###get data
        leftTickCurrent, rightTickCurrent, timeFromStart = getters.get_data_from_sensors()

        #code only changes when tick happens
        if (timeTickUpdate_bool==True):
            #get updates
            forward_velocity = updaters.update_current_linear_velocity()
            angular_velocity = updaters.update_current_angular_velocity() #* 57.29 # from radians to degrees
            distance_travelled = updaters.update_distance_travelled()
            angle_total = updaters.update_theta_travelled()
            theta = getters.get_current_theta()
            distance_to_wall= getters.get_distance_to_wall()
            current_x, change_x = updaters.update_xposition()
            current_y, change_y = updaters.update_yposition()
            
            ###Graph stuff
            updaters.update_graph_data()

            #current function being completed
            ##SISO 1.1
            #reach_theta_travelled_0_forward_vel(SISO_in_1_1)
            ##SISO 1.2
            #reach_distance_0_angular_vel(SISO_in_1_2)
            ##SISO 1.3
            #reach_coordinates_constantVelocity(SISO_in_1_3_list,SISO_in_1_3_velocity)
            ##SISO 1.4
            #current_target= reach_following_coordinates(SISO_in_1_4_list,SISO_in_1_4_velocity, SISO_in_1_4_distance, SISO_in_1_4_numPoints)
            ##MIMO 2.1
            #current_target= reach_following_coordinates(MIMO_in_1_1_list,MIMO_in_1_1_velocity,MIMO_in_1_1_sensetivity)

            #PASTE HERE
        

        if globalLoopCounter % 500 == 0:
            print("\nLinear velocity: ", str(round(forward_velocity, 5)))
            print("Angular velicity: ", str(round(angular_velocity, 5)))
            print("Angle: ", str(round(theta/2/np.pi*360, 5)))
            print("x position: ", str(round(current_x,5)))
            print("y position: ", str(round(current_y,5)))
            print("current target: ", current_target) # infCoordList
            print("distance to target", getters.get_distance_to_coordinate(current_target))
            print("total distance travelled", str(round(distance_travelled,5)))
            print("total angular displacement", str(round(angle_total,5)))
            
        #basic turn off    
        if timeFromStart > 20:
            robotRunning = False

        #PASTE TURN OFFS HERE    


        globalLoopCounter += 1
    tb.stop()
            
    ###outputs when out of loop, can be commented out
    graph_plotting.show_output_results()