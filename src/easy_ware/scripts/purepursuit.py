#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from hdl_localization.msg import ScanMatchingStatus  # status 토픽 메시지 타입

import tf
from tf import TransformListener
import sys
import math

k = 0.3  # look forward gain
Kp = 1.0  # speed proportional gain
WB = 0.9  # [m] wheel base of vehicle
Lfc = 3.0  # [m] look-ahead distance
target_speed = 4.0 / 3.6  # [m/s]
max_steering_angle = math.radians(32.5)
min_steering_angle = -math.radians(32.5)


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.line_pub = rospy.Publisher('/target_line_marker', Marker, queue_size=10)
        self.pose_sub = rospy.Subscriber('/target_marker_array', Marker, self.pose_callback)
        self.obstacle_sub = rospy.Subscriber('/livox_frame_processing/obstacle_detection', Float64MultiArray, self.obstacle_callback)
        self.status_sub = rospy.Subscriber('/status', ScanMatchingStatus, self.status_callback)  # status 토픽 구독

        self.pose_x = None
        self.velocity = target_speed
        self.pose_y = None
        self.pose_yaw = None
        self.control_steering = 0
        self.control_accel = 0
        self.previous_pose_x = None
        self.previous_pose_y = None
        self.previous_pose_yaw = None
        self.current_time = None
        self.previous_time = None
        self.control_msg = Twist()
        self.control_msg.linear.x = 0.0
        self.plus_max_speed = target_speed  # 전진 최대 속도 설정
        self.minus_max_speed = -target_speed  # 후진 최대 속도 설정
        self.control_msg.angular.z = 0.0
        self.corner_speed_ratio = 0.3
        self.goal_x = 0
        self.goal_y = 0
        self.goal_reached = False
        self.tf = TransformListener()
        self.obstacle_detected = False  # 장애물 감지 상태 플래그
        self.previous_delta = 0.0  # 이전 delta 값 초기화
        self.inlier_fraction = 0.0  # status inlier_fraction 값 저장

    def status_callback(self, status_msg):
        self.inlier_fraction = status_msg.inlier_fraction
        #rospy.loginfo("Received inlier_fraction: {}".format(self.inlier_fraction))

    def pose_callback(self, msg):
        (trans, rot) = self.tf.lookupTransform("/world", "/base_link", rospy.Time(0))
        quaternion = (rot[0], rot[1], rot[2], rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.pose_x = trans[0]
        self.pose_y = trans[1]
        self.pose_yaw = yaw

        current_time = rospy.Time.now()
        if self.previous_time is not None:
            elapsed_time = (current_time - self.previous_time).to_sec()
            distance = self.calculate_distance(self.previous_pose_x, self.previous_pose_y, self.pose_x, self.pose_y)
            self.velocity = distance / elapsed_time * 0.1 + self.velocity * 0.9
        else:
            elapsed_time = 0
            self.velocity = 0

        self.previous_time = current_time
        self.previous_pose_x = self.pose_x
        self.previous_pose_y = self.pose_y
        self.previous_pose_yaw = self.pose_yaw

        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        alpha = math.atan2(target_y - self.pose_y, target_x - self.pose_x) - self.pose_yaw
        current_delta = math.atan2(3.0 * WB * math.sin(alpha), 1.0)
        delta = current_delta 

        marker = self.make_line_strip()
        marker_length = math.floor(alpha * 100)

        self.previous_delta = delta
        
        angle = 0
        for i in range(max(10, marker_length)):
            p = Point()
            angle += delta / (i + 1.1) / 10
            p.x += i * math.cos(angle)
            p.y += i * math.sin(angle)
            p.z = 1.5
            marker.points.append(p)
        self.line_pub.publish(marker)

        if not self.obstacle_detected:
            # if self.inlier_fraction >= 0.90:
             self.control_steering = self.velocity / WB * math.tan(delta) * elapsed_time
             self.control_accel = self.proportional_control(target_speed - self.corner_speed_ratio * abs(delta), self.velocity)
             self.control_msg.linear.x = self.control_msg.linear.x + self.control_accel * elapsed_time

             if self.control_msg.linear.x > self.plus_max_speed:
                 self.control_msg.linear.x = self.plus_max_speed
             elif self.control_msg.linear.x < self.minus_max_speed:
                 self.control_msg.linear.x = self.minus_max_speed

             self.control_msg.angular.z = self.control_steering
            # else:
            #  self.control_msg.linear.x = 0.0
            #  self.control_msg.angular.z = 0.0
        else:
            self.control_msg.linear.x = 0.0
            self.control_msg.angular.z = 0.0

        self.check_goal_reached(target_x, target_y, self.pose_x, self.pose_y)
        if not self.goal_reached:
            self.cmd_pub.publish(self.control_msg)
        else:
            self.control_msg.linear.x = 0.0
            self.control_msg.angular.z = 0.0
            self.cmd_pub.publish(self.control_msg)

    def obstacle_callback(self, msg):
        if msg.data and msg.data[0] == 1.0:
            self.obstacle_detected = True
            rospy.loginfo("Obstacle detected, stopping vehicle.")
        else:
            self.obstacle_detected = False

    def check_goal_reached(self, target_x, target_y, pose_x, pose_y):
        self.distance = self.calculate_distance(pose_x, pose_y, target_x, target_y)
        if self.distance < 0.35:
            self.goal_reached = True
            rospy.loginfo("Goal reached!")

    def calculate_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def proportional_control(self, target, current):
        return Kp * (target - current)

    def make_line_strip(self):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "line_strip"
        marker.id = 0
        marker.scale.x = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.points = []
        return marker

def main():
    pure_pursuit = PurePursuit()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    sys.exit(main())
