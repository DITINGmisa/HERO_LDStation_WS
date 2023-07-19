#!/usr/bin/env python
 
import roslaunch
import rospy
from std_srvs.srv import Empty, EmptyResponse
from rosbag import Bag
 
class RosbagRecorder():
    def __init__(self):
        self.is_recording = False
        self.topics_to_record = []
        self.bag = None
 
        # create ROS node
        rospy.init_node('rosbag_recorder')
 
        # create ROS service for starting recording
        rospy.Service('/start_recording', Empty, self.start_recording)
 
        # create ROS service for stopping recording
        rospy.Service('/stop_recording', Empty, self.stop_recording)
 
        # create list of topics to record
        self.topics_to_record = ['/livox/lidar']
 
        # create ROS launch configuration
        self.launch_config = roslaunch.config.ROSLaunchConfig()
 
        # create ROSbag node
        rosbag_node = roslaunch.core.Node('rosbag', 'record', args='-a', name='rosbag')
 
        # add ROSbag node to launch configuration
        self.launch_config.add_node(rosbag_node)
 
        # start ROSbag node
        self.rosbag_process = roslaunch.core.processes.start_process(rosbag_node, self.launch_config)
 
    def start_recording(self, path):
        if not self.is_recording:
            rospy.loginfo('Starting recording...')
            self.is_recording = True
            self.bag = Bag(path, 'w')
            for topic in self.topics_to_record:
                rospy.loginfo('Adding topic {} to recording...'.format(topic))
                self.bag.write(topic)
        else:
            rospy.loginfo('Already recording.')
        return EmptyResponse()
 
    def stop_recording(self):
        if self.is_recording:
            rospy.loginfo('Stopping recording...')
            self.bag.close()
            self.is_recording = False
        else:
            rospy.loginfo('Not currently recording.')
        return EmptyResponse()
 
    def run(self):
        rospy.spin()
 
if __name__ == '__main__':
    recorder = RosbagRecorder()
    recorder.run()