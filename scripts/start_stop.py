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

    #TODO change name and multidigit
    print("Press 's' to send start message")
    print("Press 'p' to send stop message")
    print("Press 'q' to quit")
    print("Classes:")
    print("'1' : 'advance'")
    print("'2' : 'approve'")
    print("'3' : 'advance'")
    print("'4' : 'approve'")
    print("'5' : 'advance'")
    print("'6' : 'approve'")
    print("'7' : 'advance'")
    print("'8' : 'approve'")
    print("'9' : 'advance'")
    print("'10' : 'approve'")
    print("'11' : 'advance'")
    print("'12' : 'approve'")

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
                    msg.data = 'Advance'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'advance' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True

                elif char == '2' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'Approve'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                
                elif char == '3' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'Attention'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                
                elif char == '4' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'Deitic'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                
                elif char == '5' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'FollowMe'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                
                elif char == '6' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'GoLeft'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                
                elif char == '7' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'GoRight'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                
                elif char == '8' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'Halt'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                elif char == '9' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'MoveForward'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                elif char == '10' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'MoveInReverse'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                elif char == '11' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'Rally'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
                    print(f'', end="\r")
                    waiting_for_stop = True
                elif char == '12' and not waiting_for_stop:
                    msg = String()
                    msg.data = 'Stop'
                    start_pub_class.publish(msg)
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"<= Start msg sent for 'approve' (Waiting for stop) =>", end="\n")
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

