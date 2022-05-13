###Initialising variables
##aka don't touch these or program won't work

#Turtlebot instance
#tb = Turtlebot() 

#booleans
robotRunning = True
timeTickUpdate_bool = False

#lists
timeFromStartArray = DeadReckon_List_theta = DeadReckon_List_vel = current_target = [0,0]

#Graph plotting CORY
IMUList = abscissaList = ordinateList = ordinateList2 = [] # for plotting graph


#integers
globalLoopCounter = 1
refTickLeft = refTickRight = 0 #was None before


#floats     
current_x = current_y =  0
distance_travelled = angle_total =  0
forward_velocity = angular_velocity = 0
theta = 0
global_velocity_signal = 0

