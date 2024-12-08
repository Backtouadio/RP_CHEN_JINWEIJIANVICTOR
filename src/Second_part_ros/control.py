#!/usr/bin/python3
# -*- coding: utf-8 -*-
import roslib
import rospy
import sys
import termios
import tty
from std_msgs.msg import String
from pynput import keyboard  # You'll need to install this: pip install pynput

class ControlNode:
    def __init__(self):
        """Initialize the control node"""
        # Initialize the node
        rospy.init_node('control_node')
        
        # Create publisher for movement only
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        rospy.loginfo("Control node has started")
        
    def on_press(self, key):
        """Handle key press events"""
        try:
            if key == keyboard.Key.up:
                self.control_pub.publish("UP")
            elif key == keyboard.Key.down:
                self.control_pub.publish("DOWN")
            elif key == keyboard.Key.left:
                self.control_pub.publish("LEFT")
            elif key == keyboard.Key.right:
                self.control_pub.publish("RIGHT")
            elif key == keyboard.Key.esc:
                return False  # Stop listener
        except Exception as e:
            rospy.logerr(f"Error handling key press: {e}")
            
    def run(self):
        """Main node logic"""
        print("\nSnake Game Controls:")
        print("Arrow keys - Move snake")
        print("ESC - Quit control node")
        
        # Create key listener
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

if __name__ == '__main__':
    try:
        control_node = ControlNode()
        control_node.run()
    except rospy.ROSInterruptException:
        pass