#!/usr/bin/env python3
import tkinter
import subprocess
from tkinter import filedialog
import yaml

class CheckboxWithProcess(tkinter.Checkbutton):
    def __init__(self, master=None, node=None, script=None, **kwargs):
        super().__init__(master, **kwargs)
        self.process = None
        self.node = node
        self.script = script
        self.variable = tkinter.BooleanVar()
        self.config(variable=self.variable, command=self.on_checkbox_click)
    
    def on_checkbox_click(self):
        if self.variable.get():
            print(f"Checkbox for {self.script} is checked")
            # Run the python script
            # write if statement self.script extention is ".py":
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
            # Terminate the process if it exists
            if self.process:
                self.process.terminate()

def create_colored_frame(master, color):
    frame = tkinter.Frame(master, highlightbackground=color, highlightthickness=2)
    frame.pack(padx=3, pady=3, fill="both", expand=False)
    return frame

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
        rosbag_path_entry.delete(0, tkinter.END)
        rosbag_path_entry.insert(0, selected_rosbag_file)
        # Save the selected ROS bag file path to a YAML file
        with open("config.yaml", "w") as yaml_file:
            yaml.dump({"selected_rosbag_file": selected_rosbag_file}, yaml_file)

window = tkinter.Tk()
window.title("Easyware")
window.geometry("800x800+200+200")
window.resizable(True, True)

# Initialize the selected ROS bag file path
selected_rosbag_file = None
rosbag_process = None
is_rosbag_playing = False

# Try to read the saved ROS bag file path from a YAML file
try:
    with open("config.yaml", "r") as yaml_file:
        config = yaml.safe_load(yaml_file)
        selected_rosbag_file = config.get("selected_rosbag_file")
        print("found yaml")
except FileNotFoundError:
    print("not found yaml")
    pass

# Frame for the Mapping Algorithm checkboxes
mapping_frame = create_colored_frame(window, "gray")

# Set the title of the Mapping Algorithm frame
mapping_title_label = tkinter.Label(mapping_frame, text="Mapping&Transform", font=("Arial", 14, "bold"))
mapping_title_label.grid(row=0, column=0, columnspan=1, sticky="nsew")

checkbox1 = CheckboxWithProcess(mapping_frame, text="static_TF_publisher", node="easy_ware", script="static_tf.launch")
checkbox1.grid(row=1, column=0)

checkbox2 = CheckboxWithProcess(mapping_frame, text="point_Map_Loader", node="easy_ware", script="map_loader.launch")
checkbox2.grid(row=1, column=1)

checkbox3 = CheckboxWithProcess(mapping_frame, text="voxel_Grid_Sampler", node="easy_ware", script="points_downsample.launch")
checkbox3.grid(row=1, column=2)

# Frame for the rviz checkboxes
rviz_frame = create_colored_frame(window, "gray")

rviz_title_label = tkinter.Label(rviz_frame, text="Visualization", font=("Arial", 14, "bold"))
rviz_title_label.grid(row=0, column=0, columnspan=1, sticky="nsew")

checkbox4 = CheckboxWithProcess(rviz_frame, text="hdl_launcher", node="hdl_localization", script="hdl_localization.launch")
checkbox4.grid(row=1, column=0)

checkbox4_1 = CheckboxWithProcess(rviz_frame, text="hdl_Rviz_launcher", node="easy_ware", script="hdl_rviz.launch")
checkbox4_1.grid(row=1, column=1)

checkbox4_2 = CheckboxWithProcess(rviz_frame, text="easy_Rviz_launcher", node="easy_ware", script="easy_rviz.launch")
checkbox4_2.grid(row=1, column=2)


# Frame for the Localization Algorithm checkboxes
localization_frame = create_colored_frame(window, "gray")

# Set the title of the Localization Algorithm frame
localization_title_label = tkinter.Label(localization_frame, text="Localization Algorithm", font=("Arial", 14, "bold"))
localization_title_label.grid(row=0, column=0, columnspan=1, sticky="nsew")

checkbox5 = CheckboxWithProcess(localization_frame, text="ndt_matching",node="ndt_localizer", script="ndt_localizer.launch")
checkbox5.grid(row=1, column=0)

checkbox6 = CheckboxWithProcess(localization_frame, text="concat_lidar", node="pointcloud_concatenate",script="concat.launch")
checkbox6.grid(row=1, column=1)

checkbox7 = CheckboxWithProcess(localization_frame, text="Vector_Map_Loader", node="easy_ware", script="osm.launch")
checkbox7.grid(row=1, column=2)

checkbox8_1 = CheckboxWithProcess(localization_frame, text="Node_Pump", node="easy_ware", script="node_pump0.py")
checkbox8_1.grid(row=1, column=3)

checkbox8_2 = CheckboxWithProcess(localization_frame, text="Node_Pump(Distance_estimate)", node="easy_ware", script="node_pump.py")
checkbox8_2.grid(row=1, column=4)


# Frame for the path following checkboxes
path_frame = create_colored_frame(window, "gray")

path_title_label = tkinter.Label(path_frame, text="PathFollowing", font=("Arial", 14, "bold"))
path_title_label.grid(row=0, column=0, columnspan=1, sticky="nsew")

checkbox9 = CheckboxWithProcess(path_frame, text="Easy_simulation", node="easy_ware", script="easy_simulation.py")
checkbox9.grid(row=1, column=0)

checkbox10 = CheckboxWithProcess(path_frame, text="Easy_teleop", node="easy_ware", script="teleop.py")
checkbox10.grid(row=1, column=1)

checkbox11 = CheckboxWithProcess(path_frame, text="Target_selector", node="easy_ware", script="target_selector.py")
checkbox11.grid(row=1, column=2)

checkbox12 = CheckboxWithProcess(path_frame, text="Pure_pursuit", node="easy_ware", script="purepursuit.py")
checkbox12.grid(row=1, column=3)

# Sensor node
sensor_frame = create_colored_frame(window, "gray")

sensor_title_label1 = tkinter.Label(sensor_frame, text="Seonsor Node", font=("Arial", 14, "bold"))
sensor_title_label1.grid(row=0, column=0, columnspan=1, sticky="nsew")

sensor_title_label2 = tkinter.Label(sensor_frame, text="GPS_node", font=("Arial", 11, "bold"))
sensor_title_label2.grid(row=1, column=0, columnspan=1, sticky="nsew")

checkbox13 = CheckboxWithProcess(sensor_frame, text="UBlox", node="ublox_utils", script="ublox.launch")
checkbox13.grid(row=2, column=0)

checkbox14 = CheckboxWithProcess(sensor_frame, text="GPS_common", node="gps_common", script="utm_odometry_node")
checkbox14.grid(row=2, column=1)

sensor_title_label3 = tkinter.Label(sensor_frame, text="Lidar_node", font=("Arial", 11, "bold"))
sensor_title_label3.grid(row=3, column=0, columnspan=1, sticky="nsew")

checkbox15_1 = CheckboxWithProcess(sensor_frame, text="livox_activate", node="livox_ros_driver2", script="rviz_MID360.launch")
checkbox15_1.grid(row=4, column=0)




# Frame for the  editor checkboxes
editor_frame = create_colored_frame(window, "gray")

editor_title_label = tkinter.Label(editor_frame, text="Editor", font=("Arial", 14, "bold"))
editor_title_label.grid(row=0, column=0, columnspan=1, sticky="nsew")

checkbox15 = CheckboxWithProcess(editor_frame, text="map_editor_program", node="easy_ware", script="map_editor_program_modify.py")
checkbox15.grid(row=4, column=0)

checkbox16 = CheckboxWithProcess(editor_frame, text="gps_tf_editor_program", node="easy_ware", script="gps_tf_control.py")
checkbox16.grid(row=4, column=2)


# Frame for the estimator checkboxes
estimator_frame = create_colored_frame(window, "gray")

estimator_title_label = tkinter.Label(estimator_frame, text="Estimator", font=("Arial", 14, "bold"))
estimator_title_label.grid(row=0, column=0, columnspan=1, sticky="nsew")

checkbox17 = CheckboxWithProcess(estimator_frame, text="velocity_estimator", node="easy_ware", script="gps_velocity.py")
checkbox17.grid(row=4, column=0)


checkbox18 = CheckboxWithProcess(estimator_frame, text="base_to_world", node="easy_ware", script="base_link_position.py")
checkbox18.grid(row=6, column=0)


# Frame for the test checkboxes
estimator_frame = create_colored_frame(window, "gray")

estimator_title_label = tkinter.Label(estimator_frame, text="Obstacle_Detection", font=("Arial", 14, "bold"))
estimator_title_label.grid(row=0, column=0, columnspan=1, sticky="nsew")

checkbox19 = CheckboxWithProcess(estimator_frame, text="clustering + obstacle_detection", node="euclidean_cluster", script="fixed_angle_lidar.launch")
checkbox19.grid(row=4, column=0)


# Button to play ROS bag file
play_button = tkinter.Button(window, text="Play ROS Bag", command=play_rosbag)
play_button.pack()

# Button to stop ROS bag playback
stop_button = tkinter.Button(window, text="Stop ROS Bag", command=stop_rosbag)
stop_button.pack()

# Button to manually select ROS bag file
select_button = tkinter.Button(window, text="Select ROS Bag File", command=select_rosbag_file)
select_button.pack()

# Entry to show the selected ROS bag file path
rosbag_path_entry = tkinter.Entry(window, width=50)
rosbag_path_entry.pack()

# Set the entry widget's text if selected_rosbag_file is not None
if selected_rosbag_file:
    rosbag_path_entry.insert(0, selected_rosbag_file)

window.mainloop()
