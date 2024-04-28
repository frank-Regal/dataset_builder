# this will be used to set the datetime parameter on the ros parameter server

#!/usr/bin/env python
import rospy
from datetime import datetime
from std_msgs.msg import Empty

class ConfigParams():
    
    '''
    initialize the ros params
    '''
    def __init__(self):
        # get params
        start_topic_name = rospy.get_param('/start_topic_name', 'n/a')
        stop_topic_name = rospy.get_param('/stop_topic_name', 'n/a')
        
        # Setup subscribers
        rospy.Subscriber(stop_topic_name, Empty, self.reset_cb, queue_size=10)

        self.classname_id = self.set_class_id()
        self.timestamp = self.set_timestamp()
        self.set_filename_per_device()

    '''
    set class id
    '''
    def set_class_id(self):
        # get class id
        classname = rospy.get_param('/classname', 'na')
        classname_id_param = "/class/" + classname + "/id"
        classname_id = rospy.get_param(classname_id_param, 'na')
        rospy.set_param('/classname_id', classname_id)
        return classname_id
    
    '''
    set fileneame per device
    '''
    def set_filename_per_device(self):
        devices = rospy.get_param('/device/','n/a')
        for i, name in enumerate(devices):
            device_id = devices[name]['id']
            filename = 'S' + self.timestamp + '-D' + device_id + '-C' + self.classname_id + '.avi'
            rospy.set_param('/'  + name + '/filename', filename)

    '''
    set timestamp parameter
    '''
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
