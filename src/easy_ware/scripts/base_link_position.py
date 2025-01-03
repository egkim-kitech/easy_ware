#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

def get_transform(listener, target_frame, source_frame):
    try:
        # Get the latest available transform between the two frames
        now = rospy.Time(0)
        listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform(target_frame, source_frame, now)
        return trans, rot
    except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        rospy.logerr("Could not get transform from {} to {}".format(source_frame, target_frame))
        return None, None

def main():
    rospy.init_node('get_base_link_position')

    listener = tf.TransformListener()
    odom_pub = rospy.Publisher('/driving_course', Odometry, queue_size=10)

    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        # Get the transform from 'world' to 'base_link'
        trans, rot = get_transform(listener, 'world', 'base_link')

        if trans and rot:
            # Create Odometry message
            odom_msg = Odometry()

            # Fill in the header
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'world'  # Reference frame
            odom_msg.child_frame_id = 'base_link'  # The frame that is moving

            # Set the position (translation)
            odom_msg.pose.pose.position.x = trans[0]
            odom_msg.pose.pose.position.y = trans[1]
            odom_msg.pose.pose.position.z = trans[2]

            # Set the orientation (rotation)
            odom_msg.pose.pose.orientation.x = rot[0]
            odom_msg.pose.pose.orientation.y = rot[1]
            odom_msg.pose.pose.orientation.z = rot[2]
            odom_msg.pose.pose.orientation.w = rot[3]

            # The twist section of Odometry could be filled with velocity data,
            # here we set it to zero for simplicity.
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0

            # Publish the Odometry message
            odom_pub.publish(odom_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
