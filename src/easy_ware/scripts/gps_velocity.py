#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

class GPSVelocityListener:
    def __init__(self):
        self.target_velocity = 2.0  # Km/h
        self.limit = self.target_velocity * 3 / 100
        self.pose_x = None
        self.pose_y = None
        self.velocity = 0
        self.filter_velocity = 0
        self.int_velocity = 0
        self.previous_time = None
        self.previous_pose_x = None
        self.previous_pose_y = None
        self.order = 1
        self.candidate = 0
        self.value = 0
        self.num = 0
        self.velocity2 = 0
        self.error_percentages = []  # List to store error percentages

        rospy.init_node('gps_velocity_listener', anonymous=True)
        rospy.Subscriber("/gps", Odometry, self.gps_callback)
        
    def gps_callback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y

        current_time = rospy.Time.now()

        if self.previous_time is not None:
            elapsed_time = (current_time - self.previous_time).to_sec()
            distance = self.calculate_distance(self.previous_pose_x, self.previous_pose_y, self.pose_x, self.pose_y)
            self.velocity = distance / elapsed_time
            self.velocity2 = self.velocity * 3.6

        self.previous_time = current_time
        self.previous_pose_x = self.pose_x
        self.previous_pose_y = self.pose_y

        if self.velocity2 != 0:
            self.int_velocity = np.round(self.velocity2, 1)
            self.candidate += self.int_velocity
            self.num += 1
            self.average_velocity = self.candidate / self.num
            self.error = self.average_velocity - self.target_velocity
            self.error_percentage = (self.average_velocity - self.target_velocity) / self.target_velocity * 100
            self.error_percentages.append(self.error_percentage)  # Append error percentage to the list
        
        rospy.loginfo(f"Total Samples: num={self.num}, mean_v={self.average_velocity} km/h, error={self.error_percentage}%, current_speed={self.int_velocity}")

    def calculate_distance(self, x1, y1, x2, y2):
        return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

    def plot_histogram(self):
        plt.hist(self.error_percentages, bins=30, edgecolor='black', color='blue',range = [-25, 25])
        plt.xlim(-25, 25)
        plt.xlabel('Error Percentage (%)')
        plt.ylabel('Frequency')
        plt.title('Error Percentage Histogram')
        plt.axvline(x=-3, color='r', linestyle='--', label='-3%')
        plt.axvline(x=3, color='r', linestyle='--', label='+3%')
        plt.legend()
        plt.show()

    def start(self):
        rospy.spin()
        self.plot_histogram()

if __name__ == '__main__':
    listener = GPSVelocityListener()
    listener.start()
