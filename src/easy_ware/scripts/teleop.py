#!/usr/bin/env python3
import rospy
import sys
import tty
import termios
import signal
from geometry_msgs.msg import Twist
import select

class Teleop:
    def __init__(self):
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.init_keyboard_listener()
        #print("Press q to quit\n")
        #print("wsad to move\n")

    def init_keyboard_listener(self):
        # Save terminal settings
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        # Set terminal to raw mode
        tty.setraw(self.fd)

    def read_key(self):
        # Check if there is input available
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            # Read a single character from terminal
            return sys.stdin.read(1)
        else:
            return None

    def on_key_press(self, key):
        if key == 'w':
            self.twist.linear.x += 1.0
        elif key == 's':
            self.twist.linear.x += -1.0
        elif key == 'a':
            self.twist.angular.z += 0.1
        elif key == 'd':
            self.twist.angular.z += -0.1

        #print("linear: {}, angular: {}".format(self.twist.linear.x, self.twist.angular.z))
    def run(self):
        while not rospy.is_shutdown():
            key = self.read_key()
            if key == 'q':
                rospy.signal_shutdown('Quit requested by user')
                break
            elif key:
                self.on_key_press(key)
                self.pub.publish(self.twist)
            self.rate.sleep()

def signal_handler(sig, frame):
    rospy.signal_shutdown('Ctrl+C pressed')

def main():
    teleopnode = Teleop()
    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)
    try:
        teleopnode.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Restore terminal settings
        termios.tcsetattr(teleopnode.fd, termios.TCSADRAIN, teleopnode.old_settings)

if __name__ == "__main__":
    main()
