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

class CheckboxWithProcess(tk.Checkbutton):
    def __init__(self, master=None, node=None, script=None, entry=None, **kwargs):
        super().__init__(master, **kwargs)
        self.process = None
        self.node = node
        self.script = script
        self.entry = entry
        self.variable = tk.BooleanVar()
        self.config(variable=self.variable, command=self.on_checkbox_click)
    
    def on_checkbox_click(self):
        if self.variable.get():
            print(f"Checkbox for {self.script} is checked")
            try:
                if self.script.endswith(".launch"):
                    self.process = subprocess.Popen(["roslaunch", self.node, self.script])
                elif self.script.endswith(".py"):
                    self.process = subprocess.Popen(["rosrun", self.node, self.script])
                else:
                    self.process = subprocess.Popen(["rosrun", self.node, self.script])
            except Exception as e:
                print(f"Error running {self.script}: {e}")
        else:
            print(f"Checkbox for {self.script} is unchecked")
            if self.process:
                self.process.terminate()

def create_colored_frame(master, color):
    frame = tk.Frame(master, highlightbackground=color, highlightthickness=2)
    frame.pack(padx=5, pady=5, fill="both", expand=False)
    return frame

"""
def play_rosbag():
    global rosbag_process
    if selected_rosbag_file:
        rosbag_process = subprocess.Popen(["rosbag", "play", selected_rosbag_file])

def stop_rosbag():
    global rosbag_process
    if rosbag_process:
        rosbag_process.terminate()

def select_rosbag_file():
    global selected_rosbag_file
    rosbag_file = filedialog.askopenfilename(title="Select ROS Bag File", filetypes=(("ROS Bag Files", "*.bag"), ("All files", "*.*")))
    if rosbag_file:
        selected_rosbag_file = rosbag_file
        rosbag_path_entry.delete(0, tk.END)
        rosbag_path_entry.insert(0, selected_rosbag_file)
        with open("config.yaml", "w") as yaml_file:
            yaml.dump({"selected_rosbag_file": selected_rosbag_file}, yaml_file)
"""

def apply_transformations():
    global x, y, yaw
    global x_, y_, yaw_
    x_ = x + x_
    y_ = y + y_
    yaw_ = yaw + yaw_
    
    yaw_angle = np.radians(yaw_)
    print("x: ", x_, "y: ", y_, "yaw: ", yaw_angle)
    rospy.init_node('pcd_transform_broadcaster', anonymous=True)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "modify"
    t.transform.translation.x = x_
    t.transform.translation.y = y_
    t.transform.translation.z = 0.0
    q = quaternion_from_euler(0, 0, yaw_angle) # roll, pitch, yaw
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform((x_, y_, 0), q, rospy.Time.now(), "modify", "map")
    rate.sleep()
    x = 0.0
    y = 0.0
    yaw = 0.0

def send_text():
    text = new_TF_entry.get()
    rospy.init_node('send_text_node')
    rospy.Publisher('text_topic', String, queue_size=10).publish(text)

map_loader_process = subprocess.Popen(["roslaunch", "easy_ware", "map_modify_loader.launch"])
print("map_loader.launch has been started")

selected_rosbag_file = None
rosbag_process = None

x = 0.0
y = 0.0
yaw = 0.0 

x_ = 0.0
y_ = 0.0
yaw_ = 0.0 

try:
    with open("config.yaml", "r") as yaml_file:
        config = yaml.safe_load(yaml_file)
        selected_rosbag_file = config.get("selected_rosbag_file")
        print("found yaml")
except FileNotFoundError:
    print("not found yaml")
    pass

window = tk.Tk()
window.title("Map Editor")
window.geometry("700x500+100+100")
window.resizable(True, True)

new_TF_frame = create_colored_frame(window, "gray")

new_TF_label = tk.Label(new_TF_frame, text="New Static TF", font=("Arial", 14, "bold"))
new_TF_label.grid(row=0, column=1, sticky="w")

# Entry for typing text
new_TF_entry = tk.Entry(new_TF_frame)
new_TF_entry.grid(row=1, column=1)

# Send button
send_button = tk.Button(new_TF_frame, text="Send", command=send_text)
send_button.grid(row=1, column=2)

checkbox1 = CheckboxWithProcess(new_TF_frame, text="New_Static_TF", node="easy_ware", script="make_static_TF_launch.py", entry=new_TF_entry)
checkbox1.grid(row=1, column=0)

checkbox2 = CheckboxWithProcess(new_TF_frame, text="gps_common", node="gps_common", script="utm_odometry_node")
checkbox2.grid(row=1, column=5)




# map_control_frame
control_frame = create_colored_frame(window, "gray")

control_label = tk.Label(control_frame, text="Control", font=("Arial", 14, "bold"))
control_label.grid(row=0, column=1, sticky="w")

direction_frame = tk.Frame(control_frame)
direction_frame.grid(row=3, column=0, columnspan=3, pady=10)

def move_left():
    global x
    x -= 0.5
    print(f"Moved Left: x={x}, y={y}")

def move_right():
    global x
    x += 0.5
    print(f"Moved Right: x={x}, y={y}")

def move_up():
    global y
    y += 0.5
    print(f"Moved Up: x={x}, y={y}")

def move_down():
    global y
    y -= 0.5
    print(f"Moved Down: x={x}, y={y}")

def yaw_left():
    global yaw
    yaw += 0.5
    print(f"Yaw Left: yaw={yaw}")

def yaw_right():
    global yaw
    yaw -= 0.5
    print(f"Yaw Right: yaw={yaw}")

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

ok_button = tk.Button(direction_frame, text="OK", font=("Arial", 16), command=apply_transformations)
ok_button.grid(row=3, column=0, columnspan=5, pady=10)


"""
play_button = tk.Button(window, text="Play ROS Bag", command=play_rosbag)
play_button.pack()

stop_button = tk.Button(window, text="Stop ROS Bag", command=stop_rosbag)
stop_button.pack()

select_button = tk.Button(window, text="Select ROS Bag File", command=select_rosbag_file)
select_button.pack()

rosbag_path_entry = tk.Entry(window, width=50)
rosbag_path_entry.pack()

if selected_rosbag_file:
    rosbag_path_entry.insert(0, selected_rosbag_file)
"""
window.mainloop()
