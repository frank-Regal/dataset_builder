# this will be used to set the datetime parameter on the ros parameter server

#!/usr/bin/env python
import rospy
import os
from datetime import datetime
from std_msgs.msg import Empty

class ConfigParams():
    
    '''
    Initialize the ros params
    '''
    def __init__(self):
        # get params
        stop_topic_name = rospy.get_param('/stop_topic_name', 'n/a')
        self.outdir = rospy.get_param('/base_outdir', 'n/a')
        self.classname = rospy.get_param('/classname', 'na')

        # setup stop subscribers
        self.stop_sub = rospy.Subscriber(stop_topic_name, Empty, self.reset_cb, queue_size=10)

        # init first file names
        self.classname_id = self.set_class_id()
        self.timestamp = self.set_timestamp()
        self.set_filename_per_device()
        self.set_filename_for_bags()

        self.collection_num = 0

    '''
    Set class id
    '''
    def set_class_id(self):

        # dynamically set the class param name
        classname_id_param = "/class/" + self.classname + "/id"

        # get the current class id
        classname_id = rospy.get_param(classname_id_param, 'na')

        # set the parameter for all nodes to access
        rospy.set_param('/classname_id', classname_id)

        return classname_id
    
    '''
    Set filename for saving files on each device
    '''
    def set_filename_per_device(self):
        # get list of devices
        devices = rospy.get_param('/device/','n/a')

        # set the filenames for each device
        for i, name in enumerate(devices):
            device_id = devices[name]['id']
            filename = 'S' + self.timestamp + '-D' + device_id + '-C' + self.classname_id
            rospy.set_param('/'  + name + '/filename', filename)
            filepath = rospy.get_param('/' + name + '/filepath', 'na')
            filetype = rospy.get_param('/' + name + '/filetype', 'na')
            # Find the index where 'devices' starts
            index = filepath.find('/devices')

                # Slice the string from the index to the end
            result = filepath[index+1:]  # +1 to include the character after the '/'
            text = result + filename +  filetype + ' ' + self.classname_id
            self.save_annotations_to_file(text)

    def save_annotations_to_file(self, text):
        directory = self.outdir + '/annotations/'
        # Check if the directory exists
        if not os.path.exists(directory):
            # Create the directory if it does not exist
            os.makedirs(directory)

        # Open a file in write mode. If the file does not exist, it will be created.
        with open(directory + 'labels.csv', 'a') as file:
            # Write text to the filez
            file.write('\n'+text)

    def set_filename_for_bags(self):
        filename = 'S' + self.timestamp + '-DALL-C' + self.classname_id
        rospy.set_param('/bag_util/filename', filename)

    '''
    Set timestamp parameter
    '''
    def set_timestamp(self):
        # Get the current datetime
        now = datetime.now()

        # Format the date and time as YYMMDDHHMMSS
        formatted_time = str(now.strftime("%m%d%H%M%S"))

        # Set a parameter on the ROS parameter server
        rospy.set_param('/timestamp', formatted_time)

        return formatted_time
    
    '''
    Callback when "stop" messsage is heard
    '''
    def reset_cb(self, msg):
        
        # reset the filenames
        self.timestamp = self.set_timestamp()
        self.set_filename_per_device()
        self.set_filename_for_bags()
        
        # log
        self.collection_num += 1
        rospy.logwarn("[Collection Num: %d] Writing Finished. Filenames updated, ready for next set of data.", self.collection_num)
        
def main():
    # Initialize the ROS node
    rospy.init_node('config')
    
    # setup class object
    cfg_params = ConfigParams()

    # spin
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("config param node terminated.")
