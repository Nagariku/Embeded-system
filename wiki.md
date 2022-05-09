# Embeded system wiki
#### Greg and Corentin
 
**Milestone  functions **
As of 9th May 9:15 pm SISO LATE AND MIMO branch 

######  tick_to_rad(val)
- *Input Parameter*
Tick reading
- *Action*
Converts the encoders ticks into radians (multiplication factor provided)
- *Output*
The radian equivalent

######  get_linear_velocity()
- *Input Parameter*
None
- *Action*
Calculates from the tick reading and the recorded time the bot velocity
- *Output*
The linear velocity of the platform center point (mid point between wheels)

######  get_angular_velocity()
- *Input Parameter*
None
- *Action*
Calculates from the tick reading and the recorded time the bot velocity
- *Output*
The linear velocity of the platform center point (mid point between wheels)

######  data_to_list(listToSave)
- *Input Parameter*
List to be extended
- *Action*
Appends the recieved list with the tick reading and the time passed since start of the robot
- *Output*
The extended list

######  get_current_theta()
- *Input Parameter*
None
- *Action*
Calculates the theta angle the robot perfomed according to the reference frame (taken at the robot start position) (turning anticlockwise = positive, clockwise = negative)
- *Output*
The live theta angle

######  def get_averagesVandAngle():
- *Input Parameter*
None
- *Action*
Improves the dead reckoning system by taking an average of theta last time and velocity last time and comparing to current readings and taking an average
- *Output*
dead reckoning velocity and theta usage

######  get_distance_to_coordinate(listCoordsInput)
- *Input Parameter*
input x and y of currently targetted coordinate
- *Action*
Calculates Euclidean distance
- *Output*
ouputs said distance in meters

######  get_distance_to_wall()
- *Input Parameter*
global y coordinate
- *Action*
calculates euclidean distance to closest point of the wall
- *Output*
outputs said distance in meters


######  get_angle_to_next_coord(coord_x,coord_y)
- *Input Parameter*
input x and y of currently targetted coordinate + global x and y
- *Action*
calculates angle that is needs to be targetted to go in fastest direction
- *Output*
radians 0 to 2 pi

######  update_distance_moved()
- *Input Parameter*
None
- *Action*
Calculates distance traveled, adding the euclidian distance of the last change in position to the total traveled distance
- *Output*
None

######  update_angle_total()
- *Input Parameter*
global total angle travelled
- *Action*
updates theta travelled between current and last time tick
- *Output*
None

######  update_xposition()
- *Input Parameter*
none but uses global x position parameter
- *Action*
updates current x coordinate of robot
- *Output*
current x coordinate of robot in m

######  update_yposition()
- *Input Parameter*
none but uses global y position parameter
- *Action*
updates current y coordinate of robot
- *Output*
current x coordinate of robot in m


######  reach_correct_speed(set_LinVel)
- *Input Parameter*
The linear velocity set, aka velocity in the x axis (in the robot frame)
- *Action*
Sets the linear velocity, that is calculated from the determined error and the PID parameters
- *Output*
None

######  reach_correct_angle(set_angle)
- *Input Parameter*
The angular velocity set, aka angular in the theta axis (in the robot frame)
- *Action*
Sets the angular velocity, that is calculated from the determined error and the PID parameters. Keeps robot still
- *Output*
None

######  reach_correct_distance(set_distance)
- *Input Parameter*
Target distance needed to be moved in a straight line
- *Action*
Sets the  velocity, that is calculated from the determined error and the PID parameters. Moves robot until said distance is reached. (but doesn't stop fully yet)
- *Output*
None

######  reach_correct_angle_total(set_angle_total)
- *Input Parameter*
Target angular displacement needed to be spun in 1 place
- *Action*
Sets the  angular velocity, that is calculated from the determined error and the PID parameters. spins robot until said angular displacement is reached (but doesn't stop fully yet)
- *Output*
None

######  reach_correct_angle_signal(set_angle)
- *Input Parameter*
The angular velocity set, aka angular in the theta axis (in the robot frame)
- *Action*
Sets the angular velocity, that is calculated from the determined error and the PID parameters
- *Output*
signal to be used in input of robot

######  reachCoordinates_constantVel(input_x, input_y,constSpeed)
- *Input Parameter*
coordinate x and y, as well as speed towards it
- *Action*
calculates inputs of linear and rotational speed via PIDs to reach said coordinate ASAP
- *Output*
none

######  reach_following_coordinates(coordinateList)
- *Input Parameter*
list of coordinates in sequential order, as a set of lists of [x,y]
- *Action*
list of coordinates that will be reached infinitely with 3 speed cases
- *Output*
none

######  setVelTunings(input_Kp, input_Ki, input_Kd) AND SIMILAR
- *Input Parameter*
tuning parameters
- *Action*
sets said tuning paramters
- *Output*
none



