#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause

import curses
import math
import numpy as np
import quaternion
import rospy
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from typing import Tuple



_node_name = "teleop"
_pose_input_topic = "pose"
_pose_input_topic_type = PoseStamped
_output_topic = "teleop/command"
_pose_output_topic_type = PoseStamped
_path_output_topic_type = Path



class Movement:
    # All movement happens with respect to the body frame.
    _x_step = 0.01 # metres, x-axis movement step
    _y_step = 0.01 # metres, y-axis movement step
    _z_step = 0.01 # metres, z-axis movement step
    _roll_step = 0.6 # degrees, roll rotation step
    _pitch_step = 0.6 # degrees, pitch rotation step
    _yaw_step = 0.6 # degrees, yaw rotation step

    def __init__(self, x: int=0, y: int=0, z: int=0,
            roll: int=0, pitch: int=0, yaw: int=0) -> None:
        self._x = x
        self._y = y
        self._z = z
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw

    def x(self) -> float:
        return self._x * Movement._x_step

    def y(self) -> float:
        return self._y * Movement._y_step

    def z(self) -> float:
        return self._z * Movement._z_step

    def roll(self) -> float:
        return math.radians(self._roll * Movement._roll_step)

    def pitch(self) -> float:
        return math.radians(self._pitch * Movement._pitch_step)

    def yaw(self) -> float:
        return math.radians(self._yaw * Movement._yaw_step)



def init_pose(pose_topic: str) -> _pose_input_topic_type:
    return rospy.wait_for_message(pose_topic, _pose_input_topic_type)



def pose_from_str(p: PoseStamped, s: str) -> PoseStamped:
    new_p = PoseStamped()
    new_p.header.stamp = rospy.get_rostime()
    new_p.header.frame_id = p.header.frame_id
    e = s.split()
    if (len(e) != 7):
        rospy.logfatal("Invalid TSV line, expected 7 columns, got {}\n  {}".format(len(e), s))
        raise KeyboardInterrupt
    new_p.pose.position.x = float(e[0])
    new_p.pose.position.y = float(e[1])
    new_p.pose.position.z = float(e[2])
    new_p.pose.orientation.x = float(e[3])
    new_p.pose.orientation.y = float(e[4])
    new_p.pose.orientation.z = float(e[5])
    new_p.pose.orientation.w = float(e[6])
    return new_p



def update_pose(p: PoseStamped, m: Movement) -> PoseStamped:
    new_p = PoseStamped()
    new_p.header.stamp = rospy.get_rostime()
    new_p.header.frame_id = p.header.frame_id
    # T_HB, current pose
    T_HB = np.identity(4)
    T_HB[0, 3] = p.pose.position.x
    T_HB[1, 3] = p.pose.position.y
    T_HB[2, 3] = p.pose.position.z
    q_current = quaternion.quaternion(p.pose.orientation.w,
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z)
    T_HB[0:3, 0:3] = quaternion.as_rotation_matrix(q_current)
    # T_BBnew, move with respect to the body frame
    T_BBnew = np.identity(4)
    T_BBnew[0, 3] += m.x()
    T_BBnew[1, 3] += m.y()
    T_BBnew[2, 3] += m.z()
    q_yaw = quaternion.quaternion(math.cos(m.yaw() / 2), 0, 0, math.sin(m.yaw() / 2))
    q_pitch = quaternion.quaternion(math.cos(m.pitch() / 2), 0, math.sin(m.pitch() / 2), 0)
    q_roll = quaternion.quaternion(math.cos(m.roll() / 2), math.sin(m.roll() / 2), 0, 0)
    q_new = q_yaw * q_pitch * q_roll
    T_BBnew[0:3, 0:3] = quaternion.as_rotation_matrix(q_new)
    # T_HBnew, new pose
    T_HBnew = T_HB @ T_BBnew
    q = quaternion.from_rotation_matrix(T_HBnew[0:3, 0:3])
    new_p.pose.position.x = T_HBnew[0, 3]
    new_p.pose.position.y = T_HBnew[1, 3]
    new_p.pose.position.z = T_HBnew[2, 3]
    new_p.pose.orientation.x = q.x
    new_p.pose.orientation.y = q.y
    new_p.pose.orientation.z = q.z
    new_p.pose.orientation.w = q.w
    return new_p



def pose_to_path(pose: PoseStamped, new_pose: PoseStamped) -> Path:
    path = Path()
    path.header.stamp = rospy.get_rostime()
    path.header.frame_id = pose.header.frame_id
    path.poses.append(pose)
    path.poses.append(new_pose)
    return path



def wait_for_key(window) -> Tuple[Movement, bool]:
    m = Movement()
    quit = False
    try:
        while True:
            key = window.getkey()
            if key == "Q":
                quit = True
                break
            elif key == "w":
                m = Movement(x=1)
                break
            elif key == "s":
                m = Movement(x=-1)
                break
            elif key == "a":
                m = Movement(y=1)
                break
            elif key == "d":
                m = Movement(y=-1)
                break
            elif key == " ":
                m = Movement(z=1)
                break
            elif key == "c":
                m = Movement(z=-1)
                break
            elif key == "q":
                m = Movement(yaw=1)
                break
            elif key == "e":
                m = Movement(yaw=-1)
                break
            # elif key == "f":
            #     m = Movement(pitch=1)
            #     break
            # elif key == "r":
            #     m = Movement(pitch=-1)
            #     break
            # elif key == "x":
            #     m = Movement(roll=1)
            #     break
            # elif key == "z":
            #     m = Movement(roll=-1)
            #     break
            time.sleep(0.05)
    finally:
        curses.endwin()
    return m, quit



def print_waiting_for_pose(window, robot_name: str) -> None:
    window.clear()
    window.addstr(1, 0, "Waiting for initial pose of type {} on topic {}".format(_pose_input_topic_type, robot_name+"/"+_pose_input_topic))
    window.refresh()



def print_help(window) -> None:
    window.addstr(0,  0, "Position:")
    window.addstr(2,  0, "Orientation (w,x,y,z):")
    window.addstr(5,  0, "w/s       forwards/backwards")
    window.addstr(6,  0, "a/d       left/right")
    window.addstr(7,  0, "space/c   up/down")
    window.addstr(8,  0, "q/e       yaw left/right")
    # window.addstr(9,  0, "r/f       pitch up/down")
    # window.addstr(10, 0, "z/x       roll CCW/CW")
    window.addstr(11, 0, "Q         quit")



def print_pose_stamped(p: PoseStamped, window) -> None:
    position = [p.pose.position.x, p.pose.position.y, p.pose.position.z]
    orientation = [p.pose.orientation.w, p.pose.orientation.x,
            p.pose.orientation.y, p.pose.orientation.z]
    window.move(1, 0)
    window.clrtoeol()
    window.addstr(1, 0, "  " + " ".join(["{: 8.3f}".format(x) for x in position]))
    window.move(3, 0)
    window.clrtoeol()
    window.addstr(3, 0, "  " + " ".join(["{: 8.3f}".format(x) for x in orientation]))



def main() -> None:
    # Initialize ROS
    rospy.init_node(_node_name)
    publish_path = rospy.get_param("~publish_path", True)
    robot_name = rospy.get_param("~robot_name", "robot1")
    if publish_path:
        output_topic_type = _path_output_topic_type
    else:
        output_topic_type = _pose_output_topic_type
    pub = rospy.Publisher(_output_topic, output_topic_type, queue_size=10)
    # Initialize curses
    window = curses.initscr()
    try:
        curses.noecho()
        # Wait for the initial pose
        print_waiting_for_pose(window, robot_name)
        pose = init_pose(_pose_input_topic)
        # Main loop
        quit = False
        print_help(window)
        while not (rospy.is_shutdown() or quit):
            print_pose_stamped(pose, window)
            movement, quit = wait_for_key(window)
            new_pose = update_pose(pose, movement)
            if publish_path:
                pub.publish(pose_to_path(pose, new_pose))
            else:
                pub.publish(new_pose)
            pose = new_pose
    finally:
        curses.endwin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
