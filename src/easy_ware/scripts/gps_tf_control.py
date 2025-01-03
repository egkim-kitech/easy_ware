#!/usr/bin/env python3
import tkinter as tk
import subprocess
from tkinter import filedialog
import yaml
import open3d as o3d
import numpy as np
import os
import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import tf2_ros
from nav_msgs.msg import Odometry
import threading



# Define global variables for offsets
x = 0.0
y = 0.0
yaw = 0.0
x_offset = 0.0
y_offset = 0.0
yaw_offset = 0.0

def gps_callback(msg):
    global x_offset, y_offset, yaw_offset

    br = tf2_ros.TransformBroadcaster()

    # Transform from world to gps_raw
    t1 = TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "world"
    t1.child_frame_id = "gps_raw"
    t1.transform.translation.x = msg.pose.pose.position.x
    t1.transform.translation.y = msg.pose.pose.position.y
    t1.transform.translation.z = msg.pose.pose.position.z
    t1.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t1)

    # Transform from gps_raw to offset_gps with current offsets
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "gps_raw"
    t.child_frame_id = "offset_gps"
    t.transform.translation.x = x_offset
    t.transform.translation.y = y_offset
    t.transform.translation.z = 0.0
    q = quaternion_from_euler(0, 0, np.radians(yaw_offset))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def apply_transformations():
    global x, y, yaw
    global x_offset, y_offset, yaw_offset
    x_offset += x
    y_offset += y
    yaw_offset += yaw
    print("x: ", x_offset, "y: ", y_offset, "yaw: ", yaw_offset)

    # Reset x, y, yaw values
    x = 0.0
    y = 0.0
    yaw = 0.0

def start_ros_subscriber():
    rospy.Subscriber('/gps', Odometry, gps_callback)
    rospy.spin()

def move_left():
    global x
    x -= 0.1


def move_right():
    global x
    x += 0.1


def move_up():
    global y
    y += 0.1


def move_down():
    global y
    y -= 0.1


def yaw_left():
    global yaw
    yaw += 1.0


def yaw_right():
    global yaw
    yaw -= 1.0


# Create the main application window
window = tk.Tk()
window.title("GPS Offset Frame Controller")
window.geometry("400x300")

# Create the direction control frame
direction_frame = tk.Frame(window)
direction_frame.pack(pady=20)

# Create and place the control buttons
left_button = tk.Button(direction_frame, text="←", font=("Arial", 16), command=move_left)
left_button.grid(row=1, column=0, padx=10, pady=10)

up_button = tk.Button(direction_frame, text="↑", font=("Arial", 16), command=move_up)
up_button.grid(row=0, column=1, padx=10, pady=10)

right_button = tk.Button(direction_frame, text="→", font=("Arial", 16), command=move_right)
right_button.grid(row=1, column=2, padx=10, pady=10)

down_button = tk.Button(direction_frame, text="↓", font=("Arial", 16), command=move_down)
down_button.grid(row=2, column=1, padx=10, pady=10)

yaw_left_button = tk.Button(direction_frame, text="ccw", font=("Arial", 16), command=yaw_left)
yaw_left_button.grid(row=1, column=3, padx=10, pady=10)

yaw_right_button = tk.Button(direction_frame, text="cw", font=("Arial", 16), command=yaw_right)
yaw_right_button.grid(row=1, column=4, padx=10, pady=10)

# Create and place the OK button
ok_button = tk.Button(window, text="OK", font=("Arial", 16), command=apply_transformations)
ok_button.pack(pady=20)

# Start the ROS node and subscriber in a separate thread
rospy.init_node('gps_to_tf_broadcaster', anonymous=True)
thread = threading.Thread(target=start_ros_subscriber)
thread.start()

# Start the Tkinter event loop
window.mainloop()

