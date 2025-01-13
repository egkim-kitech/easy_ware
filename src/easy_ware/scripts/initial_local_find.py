#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hdl_localization.msg import ScanMatchingStatus
import math
from std_msgs.msg import String

# 퍼블리셔와 상태 저장 변수
initial_pose_pub = None
complete_pub = None  # /local_find_complete 퍼블리셔
current_orientation = None
status_received = False
gps_received = False
status_inlier_fraction = 0.0
is_paused = False  # 프로세스 일시 중지 상태를 관리하는 플래그

def quaternion_from_yaw(yaw):
    """Yaw(각도)를 쿼터니언으로 변환하는 함수"""
    half_yaw = yaw / 2.0
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw)
    }

def adjust_orientation_by_degrees(orientation, degrees):
    """현재 orientation에 주어진 degrees만큼 회전하여 새로운 orientation 반환"""
    current_yaw = math.atan2(2.0 * (orientation['w'] * orientation['z']), 1.0 - 2.0 * (orientation['z'] ** 2))
    new_yaw = current_yaw + math.radians(degrees)
    return quaternion_from_yaw(new_yaw)

def gps_callback(data):
    global gps_received, is_paused

    # 일시 정지 상태에서는 GPS 콜백을 수행하지 않음
    if is_paused:
        return

    gps_received = True
    
    # TransformListener 객체 생성
    listener = tf.TransformListener()
    
    # gps 데이터를 PoseStamped 형식으로 변환
    gps_pose = PoseStamped()
    gps_pose.header = data.header
    gps_pose.pose = data.pose.pose
    
    try:
        # 10초 이내에 world 프레임에서 map 프레임으로의 변환을 기다림
        listener.waitForTransform('map', 'world', rospy.Time(), rospy.Duration(10.0))
        
        # world 프레임에서 map 프레임으로 변환
        transformed_pose = listener.transformPose('map', gps_pose)
        
        # 변환된 좌표를 사용하여 /initial_pose 메시지 생성 및 발행
        publish_initial_pose(transformed_pose.pose.position.x, transformed_pose.pose.position.y)

        # 1초 대기 후 /status 값 수신
        rospy.sleep(1)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Transform error: {}".format(e))

def publish_initial_pose(x, y):
    global current_orientation, initial_pose_pub, is_paused

    # 일시 정지 상태에서는 initial_pose를 발행하지 않음
    if is_paused:
        return

    # /initial_pose 메시지 생성
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.stamp = rospy.Time.now()
    initial_pose_msg.header.frame_id = "map"
    initial_pose_msg.pose.pose.position.x = x
    initial_pose_msg.pose.pose.position.y = y
    initial_pose_msg.pose.pose.position.z = 0.0

    # current_orientation에서 +40도 회전 적용
    adjusted_orientation = adjust_orientation_by_degrees(current_orientation, 40)
    
    # current_orientation을 업데이트하여 40도 회전한 값이 누적되도록 함
    current_orientation = adjusted_orientation

    # 업데이트된 orientation을 initial_pose에 적용
    initial_pose_msg.pose.pose.orientation.x = current_orientation['x']
    initial_pose_msg.pose.pose.orientation.y = current_orientation['y']
    initial_pose_msg.pose.pose.orientation.z = current_orientation['z']
    initial_pose_msg.pose.pose.orientation.w = current_orientation['w']

    # covariance 값 설정 (36개의 항목을 제공)
    initial_pose_msg.pose.covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 99999.0
    ]

    # /initial_pose 토픽 발행
    initial_pose_pub.publish(initial_pose_msg)
    rospy.loginfo("Published to /initial_pose with yaw +40 degrees")

def publish_local_find_complete():
    global complete_pub
    rospy.loginfo("Publishing '1' to /local_find_complete")
    
    # /local_find_complete 토픽에 '1'을 발행
    complete_msg = String()
    complete_msg.data = '1'
    complete_pub.publish(complete_msg)

def status_callback(status_msg):
    global status_received, status_inlier_fraction, is_paused

    # /status 토픽에서 inlier_fraction 값 추출
    status_inlier_fraction = status_msg.inlier_fraction
    rospy.loginfo("Received inlier_fraction: {}".format(status_inlier_fraction))

    # status를 수신했다고 표시
    status_received = True

    # inlier_fraction 값이 0.94 이상이면 일시 중지
    if status_inlier_fraction >= 0.9 and not is_paused:
        rospy.loginfo("inlier_fraction is >= 0.9, pausing process.")
        publish_local_find_complete()
        is_paused = True  # 프로세스를 일시 중지 상태로 변경
    elif status_inlier_fraction < 0.9 and is_paused:
        rospy.loginfo("inlier_fraction is < 0.9, resuming process.")
        is_paused = False  # 프로세스를 다시 시작
        start_process()  # 프로세스를 재시작

def start_process():
    global gps_received, status_received
    gps_received = False
    status_received = False

    # GPS 데이터를 다시 수신받기 시작
    rospy.Subscriber('/gps', Odometry, gps_callback)
    rospy.loginfo("Waiting for GPS data...")

def gps_listener():
    global initial_pose_pub, complete_pub, current_orientation

    # 초기 orientation 설정 (요청된 값)
    current_orientation = {
        'x': 0.0,
        'y': 0.0,
        'z': 0.9019907722923769,
        'w': 0.4317553088259617
    }

    rospy.init_node('gps_listener', anonymous=True)

    # /status 토픽 구독
    rospy.Subscriber('/status', ScanMatchingStatus, status_callback)

    # /initial_pose 퍼블리셔 생성
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # /local_find_complete 퍼블리셔 생성
    complete_pub = rospy.Publisher('/local_find_complete', String, queue_size=10)

    # GPS 프로세스 시작
    start_process()

    # ROS 노드 종료까지 대기
    rospy.spin()

if __name__ == '__main__':
    gps_listener()
