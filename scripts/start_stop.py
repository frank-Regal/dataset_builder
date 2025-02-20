#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty, String
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
    start_pub_class = rospy.Publisher('/hri_cacti/dataset_capture/start/class', String, queue_size=1)
    stop_pub = rospy.Publisher('/hri_cacti/dataset_capture/stop', Empty, queue_size=1)

    print("Press 's' to send start message")
    print("Press 'p' to send stop message")
    print("Press 'q' to quit")
    print("Classes:")
    print("'1' : 'advance'")

    pair_count = 0
    waiting_for_stop = False
    # Set up terminal for non-blocking input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)

        while not rospy.is_shutdown():
            char = getch()

            if char is not None:
                if char == 's' and not waiting_for_stop:
                    start_pub.publish(Empty())
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start message sent. (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                elif char == 'p' and waiting_for_stop:
                    stop_pub.publish(Empty())
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    pair_count += 1
                    print(f"Stop message sent! (Collection #{pair_count} Complete)", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = False
                elif char == 'q':
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"Exiting... Recorded {pair_count} start-stop pairs", end="\r")
                    print(f'\n')
                    break

                elif char == '1' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'advance'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'advance' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True

            rospy.sleep(0.1)  # Small sleep to prevent CPU hogging

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

