import rospy
from jsk_rviz_plugins.msg import OverlayText

# List of topics to monitor
topics_to_monitor = [
    "/tf_static",
    "/Lidar_status",
    "/CAN_status",
    "/point_map",
    "/vector_map",
    "/localizer",
    "/localization_score",
    "/Target",
    "/Purepursuit",
    "/Obstacle_detection"
]

def check_topic_status(topic_name):
    """
    Check if a ROS topic is currently active.
    
    :param topic_name: The name of the topic to check.
    :return: True if the topic is active, False otherwise.
    """
    try:
        active_topics = rospy.get_published_topics()
        for topic, _ in active_topics:
            if topic == topic_name:
                return True
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Error checking topic '{topic_name}': {e}")
        return False

def publish_overlay_text(publisher, topic_status_dict):
    """
    Publish the monitoring status of multiple topics to RViz using OverlayText.
    
    :param publisher: The ROS publisher for OverlayText messages.
    :param topic_status_dict: A dictionary where keys are topic names and values are their statuses (True/False).
    """
    text_msg = OverlayText()
    text_msg.width = 800
    text_msg.height = 400
    text_msg.text_size = 12
    text_msg.line_width = 2
    text_msg.font = "DejaVu Sans Mono"

    # Header
    overlay_text = "Topic Monitoring Status\n\n"

    # Content
    for topic, is_active in topic_status_dict.items():
        status = "Running" if is_active else "Not Running"
        color = "green" if is_active else "red"
        overlay_text += f"{topic}: <span style='color:{color}'>{status}</span>\n"

    text_msg.text = overlay_text

    # Set foreground and background colors
    text_msg.fg_color.r = 1.0
    text_msg.fg_color.g = 1.0
    text_msg.fg_color.b = 1.0
    text_msg.fg_color.a = 1.0
    text_msg.bg_color.r = 0.0
    text_msg.bg_color.g = 0.0
    text_msg.bg_color.b = 0.0
    text_msg.bg_color.a = 0.5

    publisher.publish(text_msg)

if __name__ == "__main__":
    rospy.init_node("topic_monitoring_node", anonymous=True)
    overlay_pub = rospy.Publisher("/rviz_text_overlay", OverlayText, queue_size=10)
    rate = rospy.Rate(1)  # Check status every 1 second

    while not rospy.is_shutdown():
        topic_status_dict = {topic: check_topic_status(topic) for topic in topics_to_monitor}
        publish_overlay_text(overlay_pub, topic_status_dict)
        rate.sleep()
