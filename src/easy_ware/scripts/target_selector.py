#!/usr/bin/env python3
import tf 
from tf import TransformListener
import rospy
import sys
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class TargetSelector:
    def __init__(self):
        rospy.init_node('target_selector')
        self.target_pub = rospy.Publisher('/target_marker_array', Marker, queue_size=100)
        self.node_sub = rospy.Subscriber('/route_pump_marker_array', MarkerArray, self.node_callback)
        self.pose_sub = rospy.Subscriber('/easy_simulation_pose', PoseStamped, self.easy_simulation_pose)
        self.pose_x = None
        self.pose_y = None
        self.pose_yaw = []
        self.pose_v =[]
        self.tf = TransformListener()
        self.old_nearest_point_index = None
        self.lookupratio = 3
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.easy_simulation_pose)

    def easy_simulation_pose(self, data):
        # get transform from world to base_link frame

        # get transform from world to base_link frame
        (trans, rot) = self.tf.lookupTransform("/world", "/base_link", rospy.Time(0))
        # get yaw from quaternion
        quaternion = (
            rot[0],
            rot[1],
            rot[2],
            rot[3]
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.pose_x = trans[0]
        self.pose_y = trans[1]
        self.pose_yaw = yaw
        #print(trans,yaw)

    def node_callback(self, data):
        cx = []
        cy = []
        for i in range(len(data.markers)):
            cx.append(data.markers[i].pose.position.x)
            cy.append(data.markers[i].pose.position.y)

        if self.pose_x is None or self.pose_y is None:
            pass
        else:
            if self.old_nearest_point_index is None:
                dx = [self.pose_x - icx for icx in cx]
                dy = [self.pose_y - icy for icy in cy]    
                d = np.hypot(dx, dy)
                ind = np.argmin(d)
                self.old_nearest_point_index = ind
            else:
                ind = self.old_nearest_point_index
                distance_this_index = math.hypot((self.pose_x-cx[ind]),(self.pose_y-cy[ind]))
                while (ind + 1) < len(cx):
                    distance_next_index = math.hypot((self.pose_x-cx[ind+1]),(self.pose_y-cy[ind+1]))
                    if distance_this_index < distance_next_index:
                        break
                    ind = ind + 1 if (ind + 1) < len(cx) else ind
                    distance_this_index = distance_next_index
                self.old_nearest_point_index = ind    
                ind+=self.lookupratio
        

        if self.old_nearest_point_index is not None:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "target_node"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx[min(ind,len(cx)-1)]
            marker.pose.position.y = cy[min(ind,len(cx)-1)]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 3
            marker.scale.y = 3
            marker.scale.z = 3
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.target_pub.publish(marker)    

def main():
    target_selector = TargetSelector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
      pass

if __name__ == "__main__":
    # run main function and exit
    sys.exit(main())