#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tkinter

class CheckboxWithPose(tkinter.Checkbutton):
    def __init__(self, master=None, place=None, **kwargs):
        super().__init__(master, **kwargs)
        self.place = place
        self.variable = tkinter.BooleanVar()
        self.config(variable=self.variable, command=self.on_checkbox_click)

    def on_checkbox_click(self):
        if self.variable.get():
            print(f"Checkbox for {self.place['name']} is checked")
            self.publish_pose(self.place['pose'])
            # 1초 후에 동일한 Pose를 다시 발행
            self.after(1000, lambda: self.publish_pose(self.place['pose']))
        else:
            print(f"Checkbox for {self.place['name']} is unchecked")

    def publish_pose(self, pose_data):
        rospy.init_node('goal_point_publisher', anonymous=True)
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        pose_msg = PoseStamped()

        # Fill PoseStamped message
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = pose_data['position']['x']
        pose_msg.pose.position.y = pose_data['position']['y']
        pose_msg.pose.position.z = pose_data['position']['z']
        pose_msg.pose.orientation.x = pose_data['orientation']['x']
        pose_msg.pose.orientation.y = pose_data['orientation']['y']
        pose_msg.pose.orientation.z = pose_data['orientation']['z']
        pose_msg.pose.orientation.w = pose_data['orientation']['w']

        # Publish the message
        pub.publish(pose_msg)
        rospy.loginfo(f"Published Pose to /move_base_simple/goal: {pose_msg}")


def create_colored_frame(master, color):
    frame = tkinter.Frame(master, highlightbackground=color, highlightthickness=2)
    frame.pack(padx=5, pady=5, fill="both", expand=False)
    return frame


# Define places with Pose data
places = {
    "#1_place": {
        "name": "#1_place",
        "pose": {
            "position": {"x": -58.864620208740234, "y": 54.4979248046875, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.9999752060130921, "w": 0.007041829242035876}
        }
    },
    "#2_place": {
        "name": "#2_place",
        "pose": {
            "position": {"x": 10.14568042755127, "y": 96.30677795410156, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.6939585213655374, "w": 0.720014979444287}
        }
    }
}

# Create the main window
window = tkinter.Tk()
window.title("Goal Point Selector")
window.geometry("400x300")

# Frame for the Goal Point section
goal_frame = create_colored_frame(window, "gray")

# Set the title of the Goal Point frame
goal_title_label = tkinter.Label(goal_frame, text="Goal_point", font=("Arial", 14, "bold"))
goal_title_label.grid(row=0, column=1, sticky="w")

# Add checkboxes for the places
checkbox1 = CheckboxWithPose(goal_frame, place=places["#1_place"], text="#1_place")
checkbox1.grid(row=1, column=0)

checkbox2 = CheckboxWithPose(goal_frame, place=places["#2_place"], text="#2_place")
checkbox2.grid(row=1, column=1)

# Start the Tkinter main loop
window.mainloop()
