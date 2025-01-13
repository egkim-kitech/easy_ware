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
    frame.pack(padx=5, pady=5, fill="both", expand=False)
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
window.title("Command_program")
window.geometry("700x400+100+100")
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

# Frame for the Prepare Algorithm checkboxes
prepare_frame = create_colored_frame(window, "gray")

# Set the title of the Prepare Algorithm frame
prepare_title_label = tkinter.Label(prepare_frame, text="Prepare operation", font=("Arial", 14, "bold"))
prepare_title_label.grid(row=0, column=1, sticky="w")

checkbox1 = CheckboxWithProcess(prepare_frame, text="Static_TF", node="easy_ware", script="static_tf.launch")
checkbox1.grid(row=1, column=0)

checkbox2 = CheckboxWithProcess(prepare_frame, text="livox_frame_connect", node="easy_ware", script="livox_base_connect.py")
checkbox2.grid(row=1, column=1)

checkbox3 = CheckboxWithProcess(prepare_frame, text="can_start", node="easy_ware", script="can_start.py")
checkbox3.grid(row=1, column=2)

checkbox4 = CheckboxWithProcess(prepare_frame, text="can_transfer", node="easy_ware", script="can_control.py")
checkbox4.grid(row=1, column=3)


checkbox5 = CheckboxWithProcess(prepare_frame, text="Vector_Map_Loader", node="easy_ware", script="osm.launch")
checkbox5.grid(row=2, column=1)

checkbox6 = CheckboxWithProcess(prepare_frame, text="Node_Pump", node="easy_ware", script="node_pump0.py")
checkbox6.grid(row=2, column=2)





# Frame for the sensor Algorithm checkboxes
sensor_frame = create_colored_frame(window, "gray")

# Set the title of the seonsor Algorithm frame
mapping_title_label = tkinter.Label(sensor_frame, text="Sensor", font=("Arial", 14, "bold"))
mapping_title_label.grid(row=0, column=1, sticky="w")

checkbox7 = CheckboxWithProcess(sensor_frame, text="UBlox(gps)", node="ublox_utils", script="ublox.launch")
checkbox7.grid(row=1, column=1)

checkbox8 = CheckboxWithProcess(sensor_frame, text="GPS_common(gps)", node="gps_common", script="utm_odometry_node")
checkbox8.grid(row=1, column=2)

checkbox9 = CheckboxWithProcess(sensor_frame, text="livox_activate(lidar/imu)", node="livox_ros_driver2", script="rviz_MID360.launch")
checkbox9.grid(row=1, column=3)



# Frame for the localization checkboxes
localization_frame = create_colored_frame(window, "gray")

rviz_title_label = tkinter.Label(localization_frame, text="Localization", font=("Arial", 14, "bold"))
rviz_title_label.grid(row=0, column=1, sticky="w")

checkbox10 = CheckboxWithProcess(localization_frame, text="hdl_launcher", node="hdl_localization", script="hdl_localization.launch")
checkbox10.grid(row=2, column=0)

checkbox11 = CheckboxWithProcess(localization_frame, text="hdl_Rviz_launcher", node="easy_ware", script="hdl_rviz.launch")
checkbox11.grid(row=2, column=1)

checkbox12 = CheckboxWithProcess(localization_frame, text="Initial_position", node="easy_ware", script="initial_local_find.py")
checkbox12.grid(row=2, column=2)



# Frame for the mapping Algorithm checkboxes
mapping_frame = create_colored_frame(window, "gray")

# Set the title of the Localization Algorithm frame
localization_title_label = tkinter.Label(mapping_frame, text="Mapping", font=("Arial", 14, "bold"))
localization_title_label.grid(row=2, column=1, sticky="w")

checkbox13 = CheckboxWithProcess(mapping_frame, text="fast_lio", node="fast_lio", script="mapping_mid360.launch")
checkbox13.grid(row=3, column=0)




# Frame for the path following checkboxes
path_frame = create_colored_frame(window, "gray")

path_title_label = tkinter.Label(path_frame, text="PathFollowing", font=("Arial", 14, "bold"))
path_title_label.grid(row=0, column=1, sticky="w")

checkbox14 = CheckboxWithProcess(path_frame, text="Target_selector", node="easy_ware", script="target_selector.py")
checkbox14.grid(row=4, column=0)

checkbox15 = CheckboxWithProcess(path_frame, text="Pure_pursuit", node="easy_ware", script="purepursuit.py")
checkbox15.grid(row=4, column=1)


checkbox16 = CheckboxWithProcess(path_frame, text="Easy_simulation(option)", node="easy_ware", script="easy_simulation.py")
checkbox16.grid(row=4, column=1)

# Frame for the  editor checkboxes
editor_frame = create_colored_frame(window, "gray")

editor_title_label = tkinter.Label(editor_frame, text="Editor", font=("Arial", 14, "bold"))
editor_title_label.grid(row=0, column=1, sticky="w")

checkbox17 = CheckboxWithProcess(editor_frame, text="map_editor_program", node="easy_ware", script="map_editor_program_modify.py")
checkbox17.grid(row=4, column=0)

checkbox18 = CheckboxWithProcess(editor_frame, text="gps_tf_editor_program", node="easy_ware", script="gps_tf_control.py")
checkbox18.grid(row=4, column=2)




# Frame for the test checkboxes
estimator_frame = create_colored_frame(window, "gray")

estimator_title_label = tkinter.Label(estimator_frame, text="Obstacle_Detection", font=("Arial", 14, "bold"))
estimator_title_label.grid(row=0, column=1, sticky="w")

checkbox19 = CheckboxWithProcess(estimator_frame, text="clustering + obstacle_detection", node="euclidean_cluster", script="obstacle_detection")
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
