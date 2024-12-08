#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import rospkg
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg  # Instead of robotinfo_msgs
from std_msgs.msg import String
from std_msgs.msg import Int64

class ResultNode:
    def __init__(self):
        """Initialize the result node"""
        # Initialize the node
        rospy.init_node('result_node')
        
        # Store user information
        self.username = None
        
        # Create subscribers
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.score_sub = rospy.Subscriber('result_information', Int64, self.score_callback)
        
        rospy.loginfo("Result node has started")
        
    def user_callback(self, data):
        """Callback for receiving user information"""
        self.username = data.username
        rospy.loginfo(f"Received user information for: {self.username}")
        
    def score_callback(self, data):
        """Callback for receiving score information"""
        if self.username:
            score = data.data
            rospy.loginfo(f"Final Score for {self.username}: {score}")
            print(f"\nGame Over!")
            print(f"Player: {self.username}")
            print(f"Final Score: {score}")
        
    def run(self):
        """Main node logic"""
        rospy.spin()

if __name__ == '__main__':
    try:
        result_node = ResultNode()
        result_node.run()
    except rospy.ROSInterruptException:
        pass