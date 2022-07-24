"""

Path tracking with Stanley steering control and PID speed control.
developer : Nyoman Krisna Prebawa
Barelang 63 - BRAIL, Politeknik Negeri Batam

"""
import string
import numpy as np
import matplotlib.pyplot as plt
import sys

from scipy import dtype
import rospy
from scipy.spatial import distance
from numpy.random import uniform,normal
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from barelang63.msg import path_tracking
from barelang63.msg import coor2pathPlan
from barelang63.msg import estimate_position
from barelang63.msg import pathdata
from barelang63.msg import pose
from os import path, system, name
import time
import sys
import scipy.stats
import math
import cv2
import os
import rospy
import random
from os import system

sys.path.append("../../PathPlanning/CubicSpline/")
sys.path.append("/home/robot2/catkin_ws/PythonRobotics/PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise


k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle
path_data = np.zeros((10))
dataInY = np.zeros((10))
datapath = np.zeros((10))
dataestimate = np.zeros((0))
show_animation = True
out_pathY = []
out_pathX = []
cx = [] 
cy = []
cyaw = np.radians(20.0)
rbt_status = 0
pose_X =0
pose_Y =0 

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


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


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
    Compute index in the tr.
    0ajectory list of the target.
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

def pathplanning_data(path_data):
    global out_pathX
    global out_pathY, cx,cy

    out_pathX = list(path_data.x)
    out_pathX = list(path_data.y)
    # cubicspline(out_pathX, out)

def cubicspline(state, x, y):
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        out_pathX, out_pathY, ds=1)

    return cx, cy, cyaw
    
    # rospy.loginfo(path_data)

# def datapathY(dataInY):
#     out_pathY = dataInY.data
#     # rospy.loginfo(out_pathY)

def pathplanning_info(datapath):
    global rbt_status
    rbt_status = datapath.robotstatus

def pose_info(dataestimate):
    global pose_X, pose_Y
    pose_X = dataestimate.x
    pose_Y = dataestimate.y

def main():
    stanley = path_tracking()
    global out_pathX, out_pathY
    global pose_Y, pose_X, rbt_status

    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course

    try:
        rospy.init_node('pathtracking', anonymous=False)
     
        reset_pub = rospy.Publisher("update_odometri",String, queue_size=10)
        rospy.Subscriber("path_pub", pathdata, pathplanning_data)
        rospy.Subscriber("nilai_odometry", pose, pose_info)
        rospy.Subscriber("update_path", coor2pathPlan, pathplanning_info)

        # rospy.loginfo(cx[target_idx])
        pub = rospy.Publisher("path_tracking", path_tracking, queue_size=1)
        rate = rospy.Rate(45)
        print"Done Init & waiting path.."
    except:
        print"Failed you must check program"

#     pathX=[400.56145615, 550, 650, 400, 180, 300, 400]
#     pathY=[600, 650, 1020, 1100, 1020, 600, 980]
    state = State(Posex=pose_X, Posey=pose_Y, Poseyaw=np.radians(20.0), velocity=0.0)
    cx, cy, cyaw = cubicspline(state, out_pathX, out_pathY)
    target_speed = 100 / 3.6  # [m/s]

    max_simulation_time = 100.0

    # Initial state
    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while not rospy.is_shutdown():
        while max_simulation_time >= time and last_idx > target_idx and rbt_status == 1:
            ai = pid_control(target_speed, state.v)
            di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
            state.update(ai, di)
            time += dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            stanley.x = cx[target_idx]
            stanley.y = cy[target_idx]
            pub.publish(stanley)
            rospy.loginfo(out_pathX)

            rate.sleep()
            # if show_animation:  # pragma: no cover
            #     plt.cla()
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #             lambda event: [exit(0) if event.key == 'escape' else None])
            #     plt.plot(cx, cy, ".r", label="course")
            #     plt.plot(x, y, "-b", label="trajectory")
            #     plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            #     plt.axis("equal")
            #     plt.grid(True)
            #     plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            #     plt.pause(0.001)

        # Test
        # assert last_idx >= target_idx, "Cannot reach goal"

if __name__ == '__main__':
    main()
