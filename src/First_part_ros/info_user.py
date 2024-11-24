#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import rospkg
import time
from std_msgs.msg import String, Bool
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg  # Instead of robotinfo_msgs
import sys
import termios
import tty

class InfoUserNode:
    def __init__(self):
        """Initialize the info user node"""
        # Initialize the node
        rospy.init_node('info_user')
        
        # Create publisher
        self.user_pub = rospy.Publisher('user_information', user_msg, queue_size=10)
        
        rospy.loginfo("Info user node has started")
        
    def get_user_info(self):
        """Get user information from terminal"""
        print("Please enter your information:")
        name = input("Name: ")
        username = input("Username: ")
        while True:
            try:
                age = int(input("Age: "))
                break
            except ValueError:
                print("Please enter a valid age (number)")
                
        return name, username, age
        
    def run(self):
        """Main node logic"""
        # Get user information
        name, username, age = self.get_user_info()
        
        # Create message
        msg = user_msg()
        msg.name = name
        msg.username = username
        msg.age = age
        
        # Publish message
        self.user_pub.publish(msg)
        rospy.loginfo("User information sent")
        
        # Keep node running
        rospy.spin()

if __name__ == '__main__':
    try:
        info_node = InfoUserNode()
        info_node.run()
    except rospy.ROSInterruptException:
        pass
