#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2022 Senne Van Baelen
#
# SPDX-License-Identifier: Apache-2.0

"""
Turtlebot control example
"""

# pylint: disable=wrong-import-position:
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
# pylint: disable=import-error, unused-import
from rolab_tb.turtlebot import Turtlebot
import json

# =====================================================================
# Config
# =====================================================================

D_CONFIG = {
        "log_level": "DEBUG",
        "debug-logfile": None,    # debug log filepath
        "sub-msgs-ignore-check": [],
        }

lidarList = []

INIT_POSE = [0, 0, 0]

WAYPOINTS = [[0, 1], [2, 1], [0, 0]]

# angle does not matter at this point
GOAL_TOLERANCE = [0.2, 0.2]

# give a constant forward velocity, only control angular velocity
INIT_CONTROL_IN = [0.15, 0]

D_RES = {
        "description": None,
        "l_input_linvel": [],
        "l_input_angvel": [],
        "tstart_user": None,
        "l_time_user": [],
        "l_ticks_left": [],
        "l_ticks_right": [],
        "l_linvel": [],
        "l_angvel": [],
        "l_control_linvel": [],
        "l_control_angvel": [],
        "l_odo_x": [],
        "l_odo_y": [],
        "l_odo_th": [],
        "l_err_x": [],
        "l_err_y": [],
        "l_err_th": [],
        "l_err_dist": [],
        }

# PID tuning
KP_ANGLE_GOAL = 1
# KP_DIST_POINT = 0.5

def get_robot_vels(l_d_enc, timestep, d_config):

    """ return robot velocities around the center of mass """

    try:
        w_l = l_d_enc[0]*d_config['tick2rad']/timestep
        w_r = l_d_enc[1]*d_config['tick2rad']/timestep

        v_l = w_l*d_config['wheel_radius']
        v_r = w_r*d_config['wheel_radius']

        v_bot = (v_l + v_r)/2
        w_bot = (v_r - v_l)/d_config['wheel_base']

        return [v_bot, w_bot]

    except ZeroDivisionError:
        pass

    return None


def get_odometry(d_res, vels, tb_config, wheel_vels=False):

    """ integrate one step using forward euler
        note: with absolute control values, the result is almost
              exactly the same as the absolute states, meaning that forward
              eulor suffices for this rather "simple" problem (for the
              tested trajectory)
    """

    # pose = [x, y, th]
    prev_pose = [d_res['l_odo_x'][-1],
                 d_res['l_odo_y'][-1],
                 d_res['l_odo_th'][-1]]

    wheel_r = tb_config['wheel_radius']
    wheel_b = tb_config['wheel_base']

    # require to have a least two timestamps
    step =  d_res['l_time_user'][-1] - d_res['l_time_user'][-2]
    if wheel_vels:
        d_x = step*(wheel_r/2 * (vels[0] + vels[1]) * np.cos(prev_pose[2]))
        d_y = step*(wheel_r/2 * (vels[0] + vels[1]) * np.sin(prev_pose[2]))
        d_th = step*(wheel_r/wheel_b * (vels[1] - vels[0]))
    else:
        d_x = vels[0] * np.cos(prev_pose[2]) * step
        d_y = vels[0] * np.sin(prev_pose[2]) * step
        d_th = vels[1] * step

    new_pose = [prev_pose[0] + d_x,
                prev_pose[1] + d_y,
                prev_pose[2] + d_th]

    return new_pose


def get_angvel_control_input_pid(d_res, goal_pose, tb_config):

    """ determine desired control inputs to follow line with
        P(I) controller
    """

    cur_pose = [d_res['l_odo_x'][-1],
                d_res['l_odo_y'][-1],
                d_res['l_odo_th'][-1]]

    d_x = goal_pose[0] - cur_pose[0]
    d_y = goal_pose[1] - cur_pose[1]
    # d_th = goal_pose[2] - cur_pose[2]

    # not needed here
    # dist_from_goal = np.sqrt(d_x**2 + d_y**2)

    # angle between robot and goal
    theta_goal = np.arctan2(d_y, d_x)
    d_angle = theta_goal - cur_pose[2]

    # watch out: this only works if d_angle < 2pi
    if abs(d_angle) > np.pi:
        d_angle = -(2*np.pi - abs(d_angle))

    d_res['l_err_x'].append(d_x)
    d_res['l_err_y'].append(d_y)
    # d_res['l_err_th'].append(d_th)

    #P(I) control (determine desired angular velocity of robot)
    w_in = KP_ANGLE_GOAL*d_angle

    if abs(w_in) > tb_config['max_ang_vel']:
        w_in = np.sign(w_in)*tb_config['max_ang_vel']

    return w_in

def initialise_result_vectors(d_res, init_pose, init_input, goal_pose):

    """ initialise all resulting vectors """

    d_res['tstart_user'] = time.time()
    d_res['l_time_user'].append(0)
    d_res['l_linvel'].append(0)
    d_res['l_angvel'].append(0)
    d_res['l_odo_x'].append(init_pose[0])
    d_res['l_odo_y'].append(init_pose[1])
    d_res['l_odo_th'].append(init_pose[2])
    # initial control inputs
    d_res['l_control_linvel'].append(init_input[0])
    d_res['l_control_angvel'].append(init_input[1])
    d_res['l_err_x'].append(init_pose[0] - goal_pose[0])
    d_res['l_err_y'].append(init_pose[0] - goal_pose[1])
    # d_res['l_err_th'].append(init_pose[2] - goal_pose[2])
    d_res['l_err_dist'].append(0)

    return d_res

def check_endgoal(latest_pose, goal_pose, goal_tolerance):

    """ returns boolean if endgoal is reached """

    goal_pose_tol = [[goal_pose[0] - goal_tolerance[0],
                     goal_pose[0] + goal_tolerance[0]],
                     [goal_pose[1] - goal_tolerance[1],
                      goal_pose[1] + goal_tolerance[1]]]


    if goal_pose_tol[0][0] < latest_pose[0] < goal_pose_tol[0][1] and \
            goal_pose_tol[1][0] < latest_pose[1] < goal_pose_tol[1][1]:
        return True

    return False

def get_distance_to_coordinate(listCoordsInput):
    delta_X= new_pose[0] - listCoordsInput[0]
    delta_Y= new_pose[1] - listCoordsInput[1]
    totalDistanceToCoord = np.sqrt(delta_Y**2+delta_X**2)
    return totalDistanceToCoord

def get_nearest_object():
    data = tbot.get_scan()
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

def main():
    global tbot, d_res

    """ main function """

    tbot = Turtlebot(D_CONFIG)
    tb_config = tbot.get_tb_config()

    # initialise
    d_res = D_RES
    prev_enc_ticks = tbot.get_encoder_ticks()

    d_res = initialise_result_vectors(d_res, INIT_POSE, INIT_CONTROL_IN,
                                      WAYPOINTS[0])
    tbot.set_control_inputs(d_res['l_control_linvel'][-1],
                            d_res['l_control_angvel'][-1])
    # tstart = time.time()
    # while (time.time() - tstart) < 5:

    wpt_idx = 0
    
    while True:

        ticks = tbot.get_encoder_ticks()
        enc = [ticks['left'], ticks['right']]
        l_d_enc = [enc[0] - prev_enc_ticks['left'],
                   enc[1] - prev_enc_ticks['right']]
        tstep_enc = ticks['timestamp'] - prev_enc_ticks['timestamp']

        # if no new enc reading is available, continue
        if tstep_enc == 0:
            time.sleep(0.01)
            continue

        # update time vector
        d_res['l_time_user'].append(time.time() - d_res['tstart_user'])

        robot_vels = get_robot_vels(l_d_enc, tstep_enc, tb_config)
        new_pose = get_odometry(d_res, robot_vels, tb_config)

        d_res['l_linvel'].append(robot_vels[0])
        d_res['l_angvel'].append(robot_vels[1])
        d_res['l_odo_x'].append(new_pose[0])
        d_res['l_odo_y'].append(new_pose[1])
        d_res['l_odo_th'].append(new_pose[2])
        lidarList.append(get_nearest_object())

        # check for goal, if reached, break out!

        

        if check_endgoal(new_pose, WAYPOINTS[wpt_idx], GOAL_TOLERANCE):
            if wpt_idx == len(WAYPOINTS) - 1:
                # make sure lists have same length
                d_res['l_control_linvel'].append(0)
                d_res['l_control_angvel'].append(0)
                break
            wpt_idx += 1

        # determine control inputs
        w_in = get_angvel_control_input_pid(d_res, WAYPOINTS[wpt_idx], tb_config)
        # keep constant linvel
        if get_distance_to_coordinate(WAYPOINTS[wpt_idx]<0.5):
            v_in = d_res['l_control_linvel'][-1]/2
        else:
            v_in = d_res['l_control_linvel'][-1]

        d_res['l_control_linvel'].append(v_in)
        d_res['l_control_angvel'].append(w_in)

        tbot.set_control_inputs(v_in, w_in)

        # set new to prev
        prev_enc_ticks = ticks

        # make some room
        time.sleep(0.01)

    tbot.stop()
    
    
    plt.plot( d_res['l_odo_x'], label="x")
    plt.plot( d_res['l_odo_y'], label="y")
    plt.plot( d_res['l_odo_th'], label="th")
    plt.plot( lidarList, label="LIDAR")
    plt.grid()
    plt.legend()
    plt.show()


if __name__ == "__main__":

    main()
