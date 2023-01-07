#! /usr/env/bin python
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Om Awighnam Astu Namo Sidham
Om Sidhirastu Tad Astu Swaha

program fix TUGAS AKHIR 

"""

import numpy as np
import matplotlib.pyplot as plt
import sys

from scipy import dtype
import rospy
from scipy.spatial import distance
from numpy.random import uniform,normal
from std_msgs.msg import String
from std_msgs.msg import UInt8
from barelang63.msg import path_tracking
from barelang63.msg import pathdata
from barelang63.msg import robotInfo
from os import path, system, name
import time
import sys
import scipy.stats
import math
import os
import rospy
import random
from os import system

sys.path.append("../../PathPlanning/CubicSpline/")
sys.path.append("/home/asus/catkin_ws/PythonRobotics/PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise


#Parameter 
k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.08  # [s] time difference
L = 50 # [m] Wheel base of vehicle = 2.6

max_steer = np.radians(30.0)  # [rad] max steering angle
path_data = np.zeros((10))
dataInY = np.zeros((10))
datapath = np.zeros((10))
data_rbtInfo = np.zeros((100))
datastart = np.zeros((10))
show_animation = True
out_pathY = []
resultX = []
resultY = []
pathX = []
pathY = []
out_pathX = []
rbt_status = 0
pose_X =0
pose_Y =0
pose_W =0
play = 0 
posisiX = []
posisiY = []
max_velocity_rbt = 0

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, Posex=0.0, Posey=0.0, Poseyaw=0.0, velocity=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = Posex
        self.y = Posey
        self.yaw = Poseyaw
        self.v = velocity

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt

# CONTROL SPEED 
def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)

# CONTROL CROSS TRACK ERROR
def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx

#NORMALIZE ANGLE ROBOT 
def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

#Waypoint path course
def pathplanning_data(path_data):
    global out_pathX
    global out_pathY

    out_pathX = list(path_data.x)
    out_pathX = list(path_data.y)
# Robot Info  
def rbtinfo_CB(data_rbtInfo):
    global play, pose_X, pose_Y, pose_W, max_velocity_rbt
    
    # ROBOT CURRENT POSITION
    pose_X = data_rbtInfo.robot_pose.x
    pose_Y = data_rbtInfo.robot_pose.y
    pose_W = data_rbtInfo.robot_pose.z
    # MAX VELOCITY ROBOT FROM BEHAVIOR
    max_velocity_rbt = data_rbtInfo.MAX_VEL

def coordinate2degree(x, y, xi, yi):
    angle = math.atan2((xi - x), (yi - y))*180/math.pi
    if(angle<0):
        angle += 360
    return angle

# DATA FROM DUMMY DATA 
def startCB(datastart):
    global play
    play = datastart.data
    
def main():
    stanley = path_tracking()
    start = UInt8()
    global out_pathX, out_pathY
    global pose_Y, pose_X, pose_W,rbt_status, resultX, resultY, play


    rospy.init_node('pathtracking', anonymous=False)
    pub = rospy.Publisher("waypoint", path_tracking, queue_size=1)

    rospy.Subscriber("path_pub", pathdata, pathplanning_data)
    rospy.Subscriber("/robot_info", robotInfo, rbtinfo_CB)
    rospy.Subscriber("start", UInt8, startCB)

    rate = rospy.Rate(50)
    rospy.loginfo("Waiting for Update Path ROS..")
   
    while not rospy.is_shutdown():
        # EXAMPLE PATH MANUAL & TARGET COURSE
        pathX = [400,509.4,586.7,629.4,610.7,561.4,400,296,202.6,168.6,184.6,238.6]
        pathY = [600,568.8,642.2,767.6,864.9,944.9,1018.3,995.6,923.6,767.6,667.5,598.2]
        """Plot an example of Stanley steering control on a cubic spline."""
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(pathX, pathY, ds=0.1)

        """ INPUT POSISI ROBOT """
        state = State(Posex=pose_X, Posey=pose_Y, Poseyaw=np.radians(pose_W), velocity=0.0)

        target_speed = max_velocity_rbt / 3.6  # [m/s]
        max_time = 100.0

        # INITIAL STATE
        last_idx = len(cx) - 1 
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        target_idx, _ = calc_target_index(state, cx, cy)
    
        while max_time >= time and last_idx > target_idx and play == 1:

            ai = pid_control(target_speed, state.v)
            di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
            state.update(ai, di)
            time += dt
            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            posisiX.append(state.x)
            posisiY.append(pose_Y)
            # rospy.loginfo(stanley)
            print(posisiX)
            #PUBLISH TRAJECTORY TRACKING  
            stanley.x = cx[target_idx]
            stanley.y = cy[target_idx]
            stanley.w = cyaw[target_idx]
            stanley.vel = state.v *3.6
            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(cx, cy, ".r", label="path after cubic spline")
                plt.plot(pathX, pathY, "xb", label="pathWaypoint")
                plt.plot(state.x, state.y, "xg", label="R")

                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

            pub.publish(stanley)
            rate.sleep()
        if last_idx == target_idx:
            play = 0
        

        # Test
            assert last_idx >= target_idx, "Cannot reach goal"
            if show_animation:
                plt.subplots(1)
                plt.plot(t, [iv * 3.6 for iv in v], "-r")
                plt.xlabel("Time[s]")
                plt.ylabel("Speed[cm/s]")
                plt.grid(True)
                

                plt.subplots(1)
                plt.plot(cx, cy, ".r", label="target waypoint after cubic spline")
                plt.plot(x, y, "-g", label="Robot position")
                plt.plot(pathX, pathY, "xb", label="waypoints")

                plt.grid(True)
                plt.axis("equal")
                plt.xlabel("x[m]")
                plt.ylabel("y[m]")
                plt.legend()

                plt.subplots(1)
                plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
                plt.grid(True)
                plt.legend()
                plt.xlabel("line length[m]")
                plt.ylabel("yaw angle[deg]")
                plt.show()


            
            

if __name__ == '__main__':
    main()

    
