import rospy

def is_topic_running(topic_name):
    """
    Check if a ROS topic is running by listing active topics and looking for the specified topic.

    :param topic_name: The name of the topic to check.
    :return: True if the topic is running, False otherwise.
    """
    try:
        active_topics = rospy.get_published_topics()
        for topic, _ in active_topics:
            if topic == topic_name:
                return True
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Error while checking topic: {e}")
        return False

if __name__ == "__main__":
    rospy.init_node("check_topic_status_node", anonymous=True)
    topic_to_check = "/tf_static"

    if is_topic_running(topic_to_check):
        print(f"The topic '{topic_to_check}' is running.")
    else:
        print(f"The topic '{topic_to_check}' is not running.")
