#!/usr/bin/env python3

import tf
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf.transformations import quaternion_from_euler

class GPSTransformer:
    def __init__(self):
        rospy.init_node('gps_to_tf_broadcaster')
        
        self.br = tf2_ros.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.odom_pub = rospy.Publisher('/offset_odom', Odometry, queue_size=10)
        
        rospy.Subscriber('/gps', Odometry, self.gps_callback)
        
    def gps_callback(self, msg):
        current_time = rospy.Time.now()
        
        # Transform from world to gps_raw
        t1 = TransformStamped()
        t1.header.stamp = current_time
        t1.header.frame_id = "world"
        t1.child_frame_id = "gps_raw"
        t1.transform.translation.x = msg.pose.pose.position.x
        t1.transform.translation.y = msg.pose.pose.position.y
        t1.transform.translation.z = msg.pose.pose.position.z
        t1.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t1)

        # Transform from gps_raw to offset_gps
        t2 = TransformStamped()
        t2.header.stamp = current_time
        t2.header.frame_id = "gps_raw"
        t2.child_frame_id = "offset_gps"
        t2.transform.translation.x = 2.5
        t2.transform.translation.y = -0.9
        t2.transform.translation.z = 0.0
        t2.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, 0))

        self.br.sendTransform(t2)

        # Transform offset_gps to world
        try:
            self.listener.waitForTransform("/world", "/offset_gps", rospy.Time(), rospy.Duration(4.0))
            (trans_world, rot_world) = self.listener.lookupTransform("/world", "/offset_gps", rospy.Time(0))
            
            # Broadcast the transformation from world to offset_gps
            t3 = TransformStamped()
            t3.header.stamp = current_time
            t3.header.frame_id = "world"
            t3.child_frame_id = "offset_gps_in_world"
            t3.transform.translation.x = trans_world[0]
            t3.transform.translation.y = trans_world[1]
            t3.transform.translation.z = trans_world[2]
            t3.transform.rotation.x = rot_world[0]
            t3.transform.rotation.y = rot_world[1]
            t3.transform.rotation.z = rot_world[2]
            t3.transform.rotation.w = rot_world[3]

            self.br.sendTransform(t3)

            # Transform offset_gps_in_world to map
            self.listener.waitForTransform("/map", "/offset_gps_in_world", rospy.Time(), rospy.Duration(4.0))
            (trans_map, rot_map) = self.listener.lookupTransform("/map", "/offset_gps_in_world", rospy.Time(0))
            
            # Create and publish Odometry message for offset_gps frame in map frame
            offset_odom = Odometry()
            offset_odom.header.stamp = current_time
            offset_odom.header.frame_id = "map"
            offset_odom.child_frame_id = "offset_gps"
            offset_odom.pose.pose.position.x = trans_map[0]
            offset_odom.pose.pose.position.y = trans_map[1]
            offset_odom.pose.pose.position.z = trans_map[2]
            offset_odom.pose.pose.orientation.x = rot_map[0]
            offset_odom.pose.pose.orientation.y = rot_map[1]
            offset_odom.pose.pose.orientation.z = rot_map[2]
            offset_odom.pose.pose.orientation.w = rot_map[3]

            # Setting covariance (example values, adjust as needed)
            offset_odom.pose.covariance = [0.01] * 36
            offset_odom.twist.twist = Twist()
            offset_odom.twist.covariance = [0.01] * 36

            self.odom_pub.publish(offset_odom)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")

if __name__ == '__main__':
    gps_transformer = GPSTransformer()
    rospy.spin()
