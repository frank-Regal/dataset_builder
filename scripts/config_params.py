# this will be used to set the datetime parameter on the ros parameter server

#!/usr/bin/env python
import rospy
from datetime import datetime
from std_msgs.msg import Empty

class ConfigParams():
    def __init__(self):
        # get params
        start_topic_name = rospy.get_param('/start_topic_name', 'n/a')
        stop_topic_name = rospy.get_param('/stop_topic_name', 'n/a')
        
        # Setup subscribers
        rospy.Subscriber(stop_topic_name, Empty, self.set_timestamp, queue_size=10)

        self.class_id = self.set_class_id()
        self.timestamp = self.set_timestamp()
        self.set_filename_per_device()

    def set_class_id(self):
        # get class id
        class_name = rospy.get_param('/class_name', 'n/a')
        class_id_param = "/class/" + class_name + "/id"
        class_id = rospy.get_param(class_id_param, 'n/a')
        rospy.set_param('/class_name_id', class_id)
        return class_id
    
    def set_filename_per_device(self):
        devices = rospy.get_param('/device/','n/a')
        print(f"device: {devices}")
        for i, name in enumerate(devices):
            device_id = devices[name]['id']
            filename = 'S' + self.timestamp + '-D' + device_id + '-C' + self.class_id
            rospy.set_param('/'  + name + '/filename', filename)

    def set_timestamp(self):
        # Get the current datetime
        now = datetime.now()

        # Format the date and time as YYMMDDHHMMSS
        formatted_time = str(now.strftime("%m%d%H%M%S"))

        # Set a parameter on the ROS parameter server
        rospy.set_param('/timestamp', formatted_time)

        return formatted_time
    
    def reset_cb(self):
        self.set_timestamp()
        

def main():
    # Initialize the ROS node
    rospy.init_node('config')
    
    # setup class object
    cfg_params = ConfigParams()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("config param node terminated.")
