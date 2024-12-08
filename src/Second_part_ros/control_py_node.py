#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import pygame
from std_msgs.msg import String, Bool
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg
from RP_CHEN_JINWEIJIANVICTOR.srv import SetGameDifficulty

class ControlNodePygame:
    def __init__(self):
        """Initialize the control node with Pygame"""
        # Initialize ROS node
        rospy.init_node('control_node_pygame')
        
        # Create publishers
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        self.pause_pub = rospy.Publisher('pause_command', String, queue_size=10)
        
        # Game state tracking
        self.is_paused = False
        self.current_phase = "phase1"
        
        # Subscribe to game phase updates
        self.phase_sub = rospy.Subscriber('game_phase', String, self.phase_callback)
        
        # Wait for the difficulty service
        rospy.wait_for_service('difficulty')
        self.set_difficulty = rospy.ServiceProxy('difficulty', SetGameDifficulty)
        
        # Initialize Pygame
        pygame.init()
        
        # Create a small window for capturing keyboard events
        # This window can be minimized as we only need it for keyboard input
        self.screen = pygame.display.set_mode((300, 400))
        pygame.display.set_caption('Game Controls')
        
        # Initialize fonts for display
        self.font = pygame.font.SysFont("Arial", 16)
        
        rospy.loginfo("Control node (Pygame) has started")
        
    def phase_callback(self, data):
        """Track current game phase"""
        self.current_phase = data.data
        
    def handle_difficulty_selection(self, key):
        """Handle difficulty selection in phase1"""
        difficulty = None
        if key == pygame.K_1:
            difficulty = "easy"
        elif key == pygame.K_2:
            difficulty = "medium"
        elif key == pygame.K_3:
            difficulty = "hard"
            
        if difficulty:
            try:
                response = self.set_difficulty(difficulty)
                if response.success:
                    rospy.loginfo(f"Difficulty set to {difficulty}")
                else:
                    rospy.logwarn("Failed to set difficulty - not in phase1")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
    
    def handle_pause_menu(self, key):
        """Handle pause menu controls"""
        if key == pygame.K_h:  # Home
            self.pause_pub.publish("HOME")
        elif key == pygame.K_r:  # Restart
            self.pause_pub.publish("RESTART")
        elif key == pygame.K_e:  # Exit
            self.pause_pub.publish("EXIT")
        elif key in [pygame.K_1, pygame.K_2, pygame.K_3]:  # Color change
            color = key - pygame.K_1 + 1  # Convert to 1, 2, or 3
            self.pause_pub.publish(f"COLOR_{color}")
            
    def draw_controls_window(self):
        """Draw the control information window"""
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
        y += 20
        draw_text("P - Pause/Unpause game", y)
        
        y += 40
        draw_text("When paused:", y)
        y += 20
        draw_text("H - Return to home", y)
        y += 20
        draw_text("R - Restart game", y)
        y += 20
        draw_text("E - Exit game", y)
        y += 20
        draw_text("1/2/3 - Change snake color", y)
        
        y += 40
        draw_text("In start screen:", y)
        y += 20
        draw_text("1 - Easy difficulty", y)
        y += 20
        draw_text("2 - Medium difficulty", y)
        y += 20
        draw_text("3 - Hard difficulty", y)
        
        y += 40
        draw_text("ESC - Quit game", y)
        
        # Draw current state
        y += 40
        draw_text(f"Current phase: {self.current_phase}", y, (0, 100, 0))
        y += 20
        if self.is_paused:
            draw_text("Status: PAUSED", y, (200, 0, 0))
        else:
            draw_text("Status: RUNNING", y, (0, 100, 0))
            
        pygame.display.flip()
        
    def run(self):
        """Main node logic"""
        clock = pygame.time.Clock()
        running = True
        
        while running and not rospy.is_shutdown():
            # Handle Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                        
                    elif event.key == pygame.K_p:  # Pause/Unpause
                        if self.current_phase == "phase2":  # Only allow pause during gameplay
                            self.is_paused = not self.is_paused
                            self.pause_pub.publish("PAUSE")
                            
                    elif self.is_paused:  # Handle pause menu controls
                        self.handle_pause_menu(event.key)
                        
                    elif self.current_phase == "phase1":  # Handle difficulty selection
                        self.handle_difficulty_selection(event.key)
                        
                    elif not self.is_paused and self.current_phase == "phase2":
                        if event.key == pygame.K_UP:
                            self.control_pub.publish("UP")
                        elif event.key == pygame.K_DOWN:
                            self.control_pub.publish("DOWN")
                        elif event.key == pygame.K_LEFT:
                            self.control_pub.publish("LEFT")
                        elif event.key == pygame.K_RIGHT:
                            self.control_pub.publish("RIGHT")
            
            # Update control window
            self.draw_controls_window()
            
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