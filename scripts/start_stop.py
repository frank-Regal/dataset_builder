#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
import sys
import tty
import termios
import select

def is_data():
    """Check if there is data waiting on stdin"""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getch():
    """Gets a single character from stdin without blocking"""
    if is_data():
        return sys.stdin.read(1)
    return None

def main():
    # Initialize ROS node
    rospy.init_node('start_stop_publisher', anonymous=True)

    # Create publishers
    start_pub = rospy.Publisher('/hri_cacti/dataset_capture/start', Empty, queue_size=1)
    stop_pub = rospy.Publisher('/hri_cacti/dataset_capture/stop', Empty, queue_size=1)

    print("Press 's' to send start message")
    print("Press 'p' to send stop message")
    print("Press 'q' to quit")

    # Set up terminal for non-blocking input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)

        while not rospy.is_shutdown():
            char = getch()

            if char is not None:
                if char == 's':
                    start_pub.publish(Empty())
                    print("\nStart message sent!")
                elif char == 'p':
                    stop_pub.publish(Empty())
                    print("\nStop message sent!")
                elif char == 'q':
                    print("\nExiting...")
                    break

            rospy.sleep(0.1)  # Small sleep to prevent CPU hogging

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

