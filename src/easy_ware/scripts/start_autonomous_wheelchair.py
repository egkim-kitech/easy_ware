#!/usr/bin/env python3

import subprocess
import time
import rospy
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# 글로벌 변수 선언
hdl_localization_process = None
start_place_coordinate = None  # Odometry 데이터를 저장할 변수
initial_pose_pub = None  # 퍼블리셔를 글로벌로 선언
local_find_complete_sub = None  # /local_find_complete 구독자 객체

def listener():
    """
    ROS node initialization and subscriber for /local_find_complete, /odom, and /node_route_complete topics
    """
    global initial_pose_pub, local_find_complete_sub

    rospy.init_node('local_find_listener', anonymous=True)

    # /initialpose 퍼블리셔 초기화
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    # Subscriber for /local_find_complete topic
    local_find_complete_sub = rospy.Subscriber('/local_find_complete', String, local_find_complete_callback)

    # Subscriber for /odom topic to save the latest Odometry data
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Subscriber for /node_route_complete topic
    rospy.Subscriber('/node_route_complete', Int32, node_route_complete_callback)


def run_roslaunch(package, launch_file):
    """
    Function to execute a roslaunch command
    """
    try:
        process = subprocess.Popen(['roslaunch', package, launch_file])
        return process
    except Exception as e:
        print(f"Failed to launch {package}/{launch_file}: {e}")
        return None

def run_rosrun(package, script):
    """
    Function to execute a rosrun command
    """
    try:
        process = subprocess.Popen(['rosrun', package, script])
        return process
    except Exception as e:
        print(f"Failed to run {package}/{script}: {e}")
        return None

def local_find_complete_callback(data):
    """
    Callback function to process data from /local_find_complete topic
    """
    global hdl_localization_process, local_find_complete_sub

    # If the message is '1', proceed to terminate hdl_localization.launch
    if data.data == '1':
        # 구독 취소
        local_find_complete_sub.unregister()
        rospy.loginfo("Unsubscribed from /local_find_complete topic")

        # Step 1: Run node_pump0.py after 1 second
        time.sleep(1)
        print("Running node_pump0.py...")
        node_pump0_process = run_rosrun('easy_ware', 'node_pump0.py')

        # Step 2: After 2 seconds, start continuously publishing the start place to /initialpose
        time.sleep(1)
        rospy.Timer(rospy.Duration(1.0), publish_initial_pose)  # 매초마다 발행

        # Step 3: Run goal_place_select.py after publishing to /initialpose
        time.sleep(2)
        print("Running goal_place_select.py...")
        goal_place_select_process = run_rosrun('easy_ware', 'goal_place_select.py')

def odom_callback(data):
    """
    Callback function to process data from /odom topic
    """
    global start_place_coordinate
    # Save the most recent Odometry data
    start_place_coordinate = data
    rospy.loginfo("Updated start_place_coordinate from /odom")

def publish_initial_pose(event=None):
    """
    Publish the saved start_place_coordinate to /initialpose topic at regular intervals
    """
    if start_place_coordinate is None:
        rospy.logwarn("No start_place_coordinate available to publish.")
        return
    
    initial_pose_msg = PoseWithCovarianceStamped()

    # Convert Odometry data to PoseWithCovarianceStamped
    initial_pose_msg.header.stamp = rospy.Time.now()
    initial_pose_msg.header.frame_id = "map"
    initial_pose_msg.pose.pose.position = start_place_coordinate.pose.pose.position
    initial_pose_msg.pose.pose.orientation = start_place_coordinate.pose.pose.orientation

    # Covariance 설정 (필요시 수정 가능)
    initial_pose_msg.pose.covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 99999.0
    ]

    # Publish the initial pose (continuously)
    initial_pose_pub.publish(initial_pose_msg)
    rospy.loginfo("Continuously publishing start_place_coordinate to /initialpose")

def node_route_complete_callback(data):
    """
    Callback function to process data from /node_route_complete topic
    """
    if data.data == 1:
        rospy.loginfo("Received '1' from /node_route_complete. Running simulation scripts.")
        
        # Step 1
        time.sleep(1)
        print("Running purepursuit.py...")
        purepursuit_process = run_rosrun('easy_ware', 'purepursuit.py')

        # Step 2
        time.sleep(1)
        print("Running target_selector.py...")
        target_selector_process = run_rosrun('easy_ware', 'target_selector.py')


if __name__ == "__main__":
    # Step : Run static_tf.launch first
    print("Running static_tf.launch...")
    static_tf_process = run_roslaunch('easy_ware', 'static_tf.launch')
    time.sleep(0.5)

    # Step : Livox lidar on
    print("Livox lidar ON...")
    static_tf_process = run_roslaunch('livox_ros_driver2', 'rviz_MID360.launch')
    time.sleep(0.5)


    # Step : Ublox on
    print("Ublox ON...")
    static_tf_process = run_roslaunch('ublox_utils', 'ublox.launch')
    time.sleep(0.5)

    # Step : gps on
    print("gps ON...")
    static_tf_process = run_rosrun('gps_common', 'utm_odometry_node')
    time.sleep(0.5)
    
    print("Obstacle detect ON...")
    static_tf_process = run_roslaunch('euclidean_cluster', 'fixed_angle_lidar.launch')
    time.sleep(0.5)


    # Step : Launch hdl_localization.launch after static_tf.launch
    print("Launching hdl_localization...")
    hdl_localization_process = run_roslaunch('hdl_localization', 'hdl_localization.launch')
    time.sleep(0.5)

    # Step : Launch hdl_rviz.launch after 1 second
    print("Launching hdl_rviz...")
    hdl_rviz_process = run_roslaunch('easy_ware', 'hdl_rviz.launch')
    time.sleep(0.5)

    # Step : Launch osm.launch after 1 second
    print("Launching osm...")
    osm_process = run_roslaunch('easy_ware', 'osm.launch')
    time.sleep(0.5)

    # Step : Run initial_local_find.py after 1 second
    print("Running initial_local_find.py...")
    initial_local_find_process = run_rosrun('easy_ware', 'initial_local_find.py')

    # Start listener for /local_find_complete, /odom, and /node_route_complete topics
    listener()

    # Keep the script alive to allow processes to continue running
    try:
        # Keep running to allow all processes to run indefinitely
        while not rospy.is_shutdown():
            time.sleep(2)
    except KeyboardInterrupt:
        print("Shutting down processes...")

        # Terminate processes if needed
        if static_tf_process:
            static_tf_process.terminate()
        if hdl_localization_process:
            hdl_localization_process.terminate()
        if hdl_rviz_process:
            hdl_rviz_process.terminate()
        if osm_process:
            osm_process.terminate()
        if initial_local_find_process:
            initial_local_find_process.terminate()

        print("All processes terminated.")
