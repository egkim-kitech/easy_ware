#!/usr/bin/env python3

import tf
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf.transformations import quaternion_from_euler

class TransformBroadcaster:
    def __init__(self):
        rospy.init_node('transform_broadcaster2')
        
        self.br = tf2_ros.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.odom_pub = rospy.Publisher('/world_livox_odom', Odometry, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        

    def odom_callback(self, msg):
        current_time = rospy.Time.now()

        # Transform from world to livox_frame
        t2 = TransformStamped()
        t2.header.stamp = current_time
        t2.header.frame_id = "map"
        t2.child_frame_id = "livox_frame2"
        t2.transform.translation.x = msg.pose.pose.position.x
        t2.transform.translation.y = msg.pose.pose.position.y
        t2.transform.translation.z = msg.pose.pose.position.z
        t2.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t2)


        try:
            # Transform from livox_frame2 to map



            # Transform from livox_in_world to gps_raw
            self.listener.waitForTransform("/world", "/livox_frame2", rospy.Time(), rospy.Duration(4.0))
            (trans_raw, rot_raw) = self.listener.lookupTransform("/world", "/livox_frame2", rospy.Time(0))
            
            # Create and publish Odometry message for livox_frame2 in gps_raw frame
            offset_odom = Odometry()
            offset_odom.header.stamp = current_time
            offset_odom.header.frame_id = "world"
            offset_odom.child_frame_id = "livox_frame_world"
            offset_odom.pose.pose.position.x = trans_raw[0]
            offset_odom.pose.pose.position.y = trans_raw[1]
            offset_odom.pose.pose.position.z = trans_raw[2]
            offset_odom.pose.pose.orientation.x = rot_raw[0]
            offset_odom.pose.pose.orientation.y = rot_raw[1]
            offset_odom.pose.pose.orientation.z = rot_raw[2]
            offset_odom.pose.pose.orientation.w = rot_raw[3]

            # Setting covariance (example values, adjust as needed)
            offset_odom.pose.covariance = [0.01] * 36
            offset_odom.twist.twist = Twist()
            offset_odom.twist.covariance = [0.01] * 36

            self.odom_pub.publish(offset_odom)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")

if __name__ == '__main__':
    try:
        transform_broadcaster = TransformBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
