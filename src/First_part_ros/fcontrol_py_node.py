#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import pygame
from std_msgs.msg import String, Bool
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg

class ControlNodePygame:
    def __init__(self):
        """Initialize the control node with Pygame"""
        # Initialize ROS node
        rospy.init_node('control_node_pygame')
        
        # Create publisher
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        # Initialize Pygame
        pygame.init()
        
        # Create a small window for capturing keyboard events
        # This window can be minimized as we only need it for keyboard input
        self.screen = pygame.display.set_mode((200, 200))
        pygame.display.set_caption('Control Window')
        
        rospy.loginfo("Control node (Pygame) has started")
        
    def run(self):
        """Main node logic"""
        clock = pygame.time.Clock()
        running = True
        
        print("Use arrow keys to control the game (Close window or press ESC to quit)")
        
        while running and not rospy.is_shutdown():
            # Handle Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                        
                    elif event.key == pygame.K_UP:
                        self.control_pub.publish("UP")
                        rospy.logdebug("Published: UP")
                        
                    elif event.key == pygame.K_DOWN:
                        self.control_pub.publish("DOWN")
                        rospy.logdebug("Published: DOWN")
                        
                    elif event.key == pygame.K_LEFT:
                        self.control_pub.publish("LEFT")
                        rospy.logdebug("Published: LEFT")
                        
                    elif event.key == pygame.K_RIGHT:
                        self.control_pub.publish("RIGHT")
                        rospy.logdebug("Published: RIGHT")
            
            # Cap the loop at 30 FPS to prevent excessive CPU usage
            clock.tick(30)
            
        # Cleanup
        pygame.quit()
        rospy.loginfo("Control node (Pygame) is shutting down")

if __name__ == '__main__':
    try:
        control_node = ControlNodePygame()
        control_node.run()
    except rospy.ROSInterruptException:
        pass