#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import sys
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg

class InfoUserNode:
    def __init__(self):
        """Initialize the info user node"""
        # Initialize the node
        rospy.init_node('info_user', disable_signals=True)
        
        # Create publisher
        self.user_pub = rospy.Publisher('user_information', user_msg, queue_size=10)
        
        rospy.loginfo("Info user node has started")
        
    def get_user_info(self):
        """Get user information from terminal"""
        print("\n=== Player Information ===")
        print("Please enter your information:")
        
        # Clear any pending input
        import termios, tty, time
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            sys.stdin.flush()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
        # Get user input with clear prompts
        sys.stdout.write("\nName: ")
        sys.stdout.flush()
        name = input()
        
        sys.stdout.write("Username: ")
        sys.stdout.flush()
        username = input()
        
        while True:
            try:
                sys.stdout.write("Age: ")
                sys.stdout.flush()
                age = int(input())
                break
            except ValueError:
                print("Please enter a valid age (number)")
                
        return name, username, age
        
    def run(self):
        """Main node logic"""
        # Small delay to let other nodes initialize
        rospy.sleep(2)
        
        try:
            # Get user information
            name, username, age = self.get_user_info()
            
            # Create message
            msg = user_msg()
            msg.name = name
            msg.username = username
            msg.age = age
            
            # Publish message
            self.user_pub.publish(msg)
            rospy.loginfo(f"User information sent: {name}, {username}, {age}")
            
            # Keep node running
            rospy.spin()
            
        except KeyboardInterrupt:
            rospy.loginfo("Info user node shutting down")
            sys.exit(0)

if __name__ == '__main__':
    try:
        info_node = InfoUserNode()
        info_node.run()
    except rospy.ROSInterruptException:
        pass