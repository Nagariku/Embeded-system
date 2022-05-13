#PID controlls
import numpy as np

#############################

sensetivityDist = 0.05


#wall cooodinate axis
wall_y_coordinate = 1.75

##SISO inputs
#SISO 1.1
SISO_in_1_1 = 2*np.pi #360 degree turn

#SISO 1.2
SISO_in_1_2 = 2 # 2 meters

#SISO 1.3
SISO_in_1_3_list = [1,1]
SISO_in_1_3_velocity = 0.05

#SISO 1.4
SISO_in_1_4_list = [-1,-1, 3/2*np.pi]
SISO_in_1_4_velocity = 0.05 #m/s
SISO_in_1_4_distance = 1 # 1 meters
SISO_in_1_4_numPoints = 2

#MIMO 1.1
MIMO_in_1_1_list =[[0,1],[2,1],[0,0]]
MIMO_in_1_1_sensetivity = 0.05 #m
MIMO_in_1_1_velocity = 0.05 #m/s

#velocity controlls
vKp = 0.75
vKi = 2.85
vKd = 0.126
#vErrorList = vTimeDifferences =np.zeros(4) #each zero = 0.045sec

#angular controls
aKp = 0.6 #0.35
aKi = 2.85
aKd = 0.126
#aErrorList = np.zeros(4) #each zero = 0.045sec
#aTimeDifferences = np.zeros(4) #each zero = 0.045sec

#distance travelled controls
dKp = 0.1
dKi = 0
dKd = 0.126

#total theta travelled controls        
aDKp = 0.3
aDKi = 0
aDKd = 0