#!/usr/bin/env python3
import rospy
import sys
import os
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped,Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler,euler_from_quaternion
import math

class EasySimulationNode:
    def __init__(self):
        # init node
        rospy.init_node('easy_simulation_node')
        self.pub = rospy.Publisher('easy_simulation_pose', PoseStamped, queue_size=10)
        # make publisher for rviz marker that show rectangular area for pose_stamped
        self.pub_recmarker = rospy.Publisher('easy_simulation_marker', Marker, queue_size=10)
        # Create a Timer to broadcast the transform periodically
        self.initial_pose = None
        self.command_vel = None
        self.command_brake = None
        self.command_steer = None
        self.dt = 0.05
        
        # Subscribe to /initialpose but only once
        self.initialpose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.posecallback)
        
        # Subscriber for geometry_msgs/Twist
        rospy.Subscriber('/cmd_vel', Twist, self.commandcallback)
        self.transform_timer = rospy.Timer(rospy.Duration(0.05), self.broadcast_transform_callback)
        

    def posecallback(self, msg):
        # Store the initial pose
        self.initial_pose = msg.pose.pose
        euler = euler_from_quaternion([self.initial_pose.orientation.x,
                                       self.initial_pose.orientation.y,
                                       self.initial_pose.orientation.z,
                                       self.initial_pose.orientation.w])
        roll, pitch, yaw = euler
        print(roll, pitch, yaw)

        # Unregister the subscriber after receiving the first message
        self.initialpose_sub.unregister()
        rospy.loginfo("Unsubscribed from /initialpose after receiving the first message.")

    def commandcallback(self, msg):
        self.command_vel = msg.linear.x
        self.command_brake = -msg.linear.y
        self.command_steer = msg.angular.z
      
    def broadcast_transform(self, msg):
        # Create a TransformBroadcaster instance
        br = tf.TransformBroadcaster()
        
        if self.command_vel is not None and self.command_steer is not None:
            # Calculate the new position based on the velocity and time
            euler = euler_from_quaternion([msg.orientation.x,
                                           msg.orientation.y,
                                           msg.orientation.z,
                                           msg.orientation.w])
            roll, pitch, yaw = euler
            # Steering update
            
            yaw += self.command_steer * self.dt
            new_x = msg.position.x + self.command_vel * math.cos(yaw) * self.dt
            new_y = msg.position.y + self.command_vel * math.sin(yaw) * self.dt
            new_z = msg.position.z

            # Convert the yaw angle to a quaternion
            quaternion = quaternion_from_euler(roll, pitch, yaw)

            # Create a transform from the map frame to the base_link frame
            br.sendTransform(
                (new_x, new_y, new_z),
                (0, 0, quaternion[2], quaternion[3]),  # Using quaternion for orientation
                rospy.Time.now(),
                "base_link",
                "map"
            )
            msg.position.x = new_x
            msg.position.y = new_y
            msg.position.z = new_z
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]

        else:
            # Create a transform from the map frame to the base_link frame
            br.sendTransform(
                (msg.position.x, msg.position.y, msg.position.z),
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                rospy.Time.now(),
                "base_link",
                "map"
            )

    def broadcast_transform_callback(self, event):
        if self.initial_pose is not None:
            self.broadcast_transform(self.initial_pose)

            # Create the PoseStamped message and publish it
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose.position.x = 0
            pose_stamped.pose.position.y = 0
            pose_stamped.pose.position.z = 0
            pose_stamped.pose.orientation.x = 0
            pose_stamped.pose.orientation.y = 0
            pose_stamped.pose.orientation.z = 0
            pose_stamped.pose.orientation.w = 0
            self.pub.publish(pose_stamped)

            # Create the Marker message and publish it
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "easy_simulation_marker"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0
            marker.scale.x = 4.0  # wheel base
            marker.scale.y = 2.0  # width
            marker.scale.z = 3.0  # Height
            marker.color.a = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.pub_recmarker.publish(marker)
        
def main():
    easysimnode = EasySimulationNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    # run main function and exit
    sys.exit(main())
