#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import numpy as np

class PointCloudPublisher:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.point_received = False
        hz = 5.0
        
        self.point_cloud_pub = rospy.Publisher('/point_obstacle', PointCloud2, queue_size=10)
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        rospy.Timer(rospy.Duration(1.0 / hz), self.timer_callback)  # 5 Hz

    def clicked_point_callback(self, data):
        self.x = data.point.x
        self.y = data.point.y
        self.z = data.point.z
        self.point_received = True
        rospy.loginfo(f"Clicked point coordinates: x={self.x}, y={self.y}")

    def timer_callback(self, event):
        interval = 0.2  # meter

        plus_x_length = int(1.0 / interval) + 1
        minus_x_length = int(-1.0 / interval)

        plus_y_length = int(1.5 / interval) + 1
        minus_y_length = int(-2.0 / interval)

        plus_z_length = int(2.0 / interval) + 1
        minus_z_length = 0

        if not self.point_received:
            return

        points = []
        for i in range(minus_x_length, plus_x_length):  # -1m to +1m with 0.05m interval in x
            for j in range(minus_y_length, plus_y_length):  # -2m to +1.5m with 0.05m interval in y
                for k in range(minus_z_length, plus_z_length):  # 0m to +2m with 0.05m interval in z
                    x = self.x + i * interval
                    y = self.y + j * interval
                    z = self.z + k * interval
                    points.append([x, y, z])

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'  # Change this to the appropriate frame

        point_cloud = pc2.create_cloud_xyz32(header, points)
        self.point_cloud_pub.publish(point_cloud)

def main():
    rospy.init_node('point_cloud_publisher', anonymous=True)
    point_cloud_publisher = PointCloudPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
