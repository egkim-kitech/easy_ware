#!/usr/bin/env python3
import rospy
import sys
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

class EasySimulationNode:
    def __init__(self):
        # init node
        rospy.init_node('easy_simulation_node')
        self.pub = rospy.Publisher('easy_simulation_pose', PoseStamped, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.transform_timer = rospy.Timer(rospy.Duration(0.05), self.broadcast_transform_callback)

    def broadcast_transform_callback(self, event):
        try:
            # Look up the transform from "map" to "livox_frame"
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/livox_frame', rospy.Time(0))
            
            # Extract yaw rotation from livox_frame
            _, _, yaw = euler_from_quaternion(rot)
            
            # Set the base_link position to match livox_frame, but with a z offset
            base_link_x = trans[0]
            base_link_y = trans[1]
            base_link_z = trans[2] - 1.5  # z offset to make livox_frame 1.5m above base_link

            # Apply -30 degree pitch rotation to base_link relative to livox_frame
            pitch_angle = 0.0 * (3.14159265359 / 180.0)  # Convert -30 degrees to radians
            rotation_offset = quaternion_from_euler(0, pitch_angle, yaw)  # Apply yaw from livox_frame

            # Broadcast the transform from base_link to map
            br = tf.TransformBroadcaster()
            br.sendTransform(
                (base_link_x, base_link_y, base_link_z),
                rotation_offset,
                rospy.Time.now(),
                "base_link",
                "map"
            )

            # Publish PoseStamped message for base_link
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose.position.x = 0
            pose_stamped.pose.position.y = 0
            pose_stamped.pose.position.z = 0
            pose_stamped.pose.orientation.x = rotation_offset[0]
            pose_stamped.pose.orientation.y = rotation_offset[1]
            pose_stamped.pose.orientation.z = rotation_offset[2]
            pose_stamped.pose.orientation.w = rotation_offset[3]
            self.pub.publish(pose_stamped)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform from map to livox_frame not available")

def main():
    easysimnode = EasySimulationNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    # run main function and exit
    sys.exit(main())
