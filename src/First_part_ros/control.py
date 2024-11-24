#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import rospkg
from std_msgs.msg import String, Bool
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg  # Instead of robotinfo_msgs
import sys
import termios
import tty

class ControlNode: #without pygame 
    def __init__(self):
        """Initialize the control node"""
        # Initialize the node
        rospy.init_node('control_node')
        
        # Create publisher
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        rospy.loginfo("Control node has started")
        
    def get_key(self):
        """Get keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
    def run(self):
        """Main node logic"""
        print("Use arrow keys to control the game (q to quit):")
        
        while not rospy.is_shutdown():
            key = self.get_key()
            
            if key == '\x1b':  # Arrow key prefix
                key = self.get_key()  # Get [ character
                key = self.get_key()  # Get actual arrow key character
                
                if key == 'A':  # Up arrow
                    self.control_pub.publish("UP")
                elif key == 'B':  # Down arrow
                    self.control_pub.publish("DOWN")
                elif key == 'C':  # Right arrow
                    self.control_pub.publish("RIGHT")
                elif key == 'D':  # Left arrow
                    self.control_pub.publish("LEFT")
                    
            elif key == 'q':
                break

if __name__ == '__main__':
    try:
        control_node = ControlNode()
        control_node.run()
    except rospy.ROSInterruptException:
        pass
