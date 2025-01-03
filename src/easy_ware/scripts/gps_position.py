#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt

class OdomListener:
    def __init__(self):
        rospy.init_node('odom_listener')

        # Initialize variables to store x and y values
        self.offset_x = None
        self.offset_y = None
        self.slam_x = None
        self.slam_y = None
        self.distances = []  # List to store distances

        # Subscribers for the /offset_odom and /odom topics
        rospy.Subscriber('/offset_odom', Odometry, self.offset_odom_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def offset_odom_callback(self, msg):
        # Extract x and y values from /offset_odom topic and store them
        self.offset_x = msg.pose.pose.position.x
        self.offset_y = msg.pose.pose.position.y
        #rospy.loginfo(f"Offset Odom x: {self.offset_x}, y: {self.offset_y}")

    def odom_callback(self, msg):
        # Extract x and y values from /odom topic and store them
        self.slam_x = msg.pose.pose.position.x
        self.slam_y = msg.pose.pose.position.y
        #rospy.loginfo(f"SLAM Odom x: {self.slam_x}, y: {self.slam_y}")

        # Calculate distance if both offset and slam data are available
        self.calculate_distance()

    def calculate_distance(self):
        if self.offset_x is not None and self.offset_y is not None and self.slam_x is not None and self.slam_y is not None:
            distance = math.sqrt((self.offset_x - self.slam_x) ** 2 + (self.offset_y - self.slam_y) ** 2)
            rospy.loginfo(f"Distance: {distance}m")
            self.distances.append(distance)  # Append distance to the list
            self.offset_x = None
            self.offset_y = None

    def plot_histogram(self):
        plt.hist(self.distances, bins=100, edgecolor='black', color='blue',range = [0, 10])
        plt.xlim(0, 10)
        plt.xlabel('Error (m)')
        plt.ylabel('Frequency')
        plt.title('Distance Error Histogram')
        plt.axvline(x=0.5, color='r', linestyle='--', label='+0.5m')
        plt.xticks([i * 0.5 for i in range(21)])
        plt.legend()
        plt.show()
        

    def start(self):
        rospy.spin()
        self.plot_histogram()

if __name__ == '__main__':
    try:
        odom_listener = OdomListener()
        odom_listener.start()
    except rospy.ROSInterruptException:
        pass
