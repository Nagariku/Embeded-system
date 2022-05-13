###Initialising variables
##aka don't touch these or program won't work

#Turtlebot instance
#tb = Turtlebot() 

#booleans
robotRunning = True
timeTickUpdate_bool = False

#lists
timeFromStartArray = [0,0]
DeadReckon_List_theta = [0,0]
DeadReckon_List_vel = [0,0]
current_target=[0,0]

#Graph plotting CORY
IMUList = []
abscissaList = [] # for plotting graph
ordinateList = [] # for plotting graph
ordinateList2 = [] # for plotting graph

#integers
globalLoopCounter = 1
refTickLeft = 0 #was None before
refTickRight = 0 #was None before

#floats     
current_x = 0
current_y = 0
theta = 0
distance_travelled = 0
angle_total = 0
forward_velocity = 0
angular_velocity = 0
global_velocity_signal = 0

