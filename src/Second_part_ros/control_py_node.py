#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import pygame
from std_msgs.msg import String
from pynput import keyboard

class ControlNodePygame:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('control_node_pygame')
        
        # Create publisher for movement only
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        # Initialize Pygame
        pygame.init()
        
        # Create a small window for displaying controls
        self.screen = pygame.display.set_mode((300, 200))
        pygame.display.set_caption('Game Controls')
        
        # Initialize fonts for display
        self.font = pygame.font.SysFont("Arial", 16)
        
        # Flag for running state
        self.running = True
        
        rospy.loginfo("Control node (Pygame) has started")
            
    def draw_controls_window(self):
        # Draw the control information window
        self.screen.fill((240, 240, 240))  # Light gray background
        
        # Helper function to render text
        def draw_text(text, y, color=(0, 0, 0)):
            surface = self.font.render(text, True, color)
            rect = surface.get_rect(left=10, top=y)
            self.screen.blit(surface, rect)
        
        # Draw control information
        y = 10
        draw_text("Snake Game Controls:", y)
        y += 30
        draw_text("Arrow keys - Move snake", y)
        y += 40
        draw_text("ESC - Quit control node", y)
            
        pygame.display.flip()
    
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
                self.running = False
                return False  # Stop listener
        except Exception as e:
            rospy.logerr(f"Error handling key press: {e}")
        
    def run(self):
        clock = pygame.time.Clock()
        
        # Start keyboard listener in a non-blocking way
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()
        
        while self.running and not rospy.is_shutdown():
            # Handle Pygame window events (for window closing)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            # Update control window
            self.draw_controls_window()
            
            # Cap the loop at 30 FPS
            clock.tick(30)
            
        # Cleanup
        listener.stop()
        pygame.quit()
        rospy.loginfo("Control node (Pygame) is shutting down")

if __name__ == '__main__':
    try:
        control_node = ControlNodePygame()
        control_node.run()
    except rospy.ROSInterruptException:
        pass