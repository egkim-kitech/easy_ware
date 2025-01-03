#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import tf_conversions
import math

def broadcast_odometry():
    rospy.init_node('tf_broadcaster_node')

    # Odometry publisher
    odom_pub = rospy.Publisher('/odom', nav_msgs.msg.Odometry, queue_size=10)

    # Transform broadcaster
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Lookup transform from livox_frame to map
            transform = tf_buffer.lookup_transform('map', 'livox_frame', rospy.Time(0))

            # Create Odometry message
            odom_msg = nav_msgs.msg.Odometry()

            # Header information
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'map'   # Fixed frame (map)
            odom_msg.child_frame_id = 'livox_frame'  # Moving frame (livox_frame)

            # Pose (position and orientation)
            odom_msg.pose.pose.position.x = transform.transform.translation.x
            odom_msg.pose.pose.position.y = transform.transform.translation.y
            odom_msg.pose.pose.position.z = transform.transform.translation.z
            odom_msg.pose.pose.orientation = transform.transform.rotation

            # Velocity (set to 0, since it's a static transform)
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0

            # Publish the odometry message
            odom_pub.publish(odom_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_odometry()
    except rospy.ROSInterruptException:
        pass
