#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import sys
import termios
import tty
from std_msgs.msg import String, Bool
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg  
from RP_CHEN_JINWEIJIANVICTOR.srv import SetGameDifficulty

class ControlNode:
    def __init__(self):
        """Initialize the control node"""
        # Initialize the node
        rospy.init_node('control_node')
        
        # Create publishers
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        self.pause_pub = rospy.Publisher('pause_command', String, queue_size=10)
        
        # Game state
        self.is_paused = False
        self.current_phase = "phase1"  # Track game phase
        
        # Subscribe to game phase updates
        self.phase_sub = rospy.Subscriber('game_phase', String, self.phase_callback)
        
        # Wait for the difficulty service
        rospy.wait_for_service('difficulty')
        self.set_difficulty = rospy.ServiceProxy('difficulty', SetGameDifficulty)
        
        rospy.loginfo("Control node has started")
        
    def phase_callback(self, data):
        """Track current game phase"""
        self.current_phase = data.data
        
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
        
    def handle_difficulty_selection(self, key):
        """Handle difficulty selection in phase1"""
        difficulty = None
        if key == '1':
            difficulty = "easy"
        elif key == '2':
            difficulty = "medium"
        elif key == '3':
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
        if key == 'h':  # Home
            self.pause_pub.publish("HOME")
        elif key == 'r':  # Restart
            self.pause_pub.publish("RESTART")
        elif key == 'e':  # Exit
            self.pause_pub.publish("EXIT")
        elif key in ['1', '2', '3']:  # Color change
            self.pause_pub.publish(f"COLOR_{key}")
            
    def run(self):
        """Main node logic"""
        print("\nSnake Game Controls:")
        print("Arrow keys - Move snake")
        print("P - Pause/Unpause game")
        print("\nWhen paused:")
        print("H - Return to home")
        print("R - Restart game")
        print("E - Exit game")
        print("1/2/3 - Change snake color")
        print("\nIn start screen:")
        print("1 - Easy difficulty")
        print("2 - Medium difficulty")
        print("3 - Hard difficulty")
        print("\nQ - Quit game")
        
        while not rospy.is_shutdown():
            key = self.get_key()
            
            if key == '\x1b':  # Arrow key prefix
                key = self.get_key()  # Get [ character
                key = self.get_key()  # Get actual arrow key character
                
                if not self.is_paused and self.current_phase == "phase2":
                    if key == 'A':  # Up arrow
                        self.control_pub.publish("UP")
                    elif key == 'B':  # Down arrow
                        self.control_pub.publish("DOWN")
                    elif key == 'C':  # Right arrow
                        self.control_pub.publish("RIGHT")
                    elif key == 'D':  # Left arrow
                        self.control_pub.publish("LEFT")
                        
            elif key == 'p' or key == 'P':  # Pause/Unpause
                if self.current_phase == "phase2":  # Only allow pause during gameplay
                    self.is_paused = not self.is_paused
                    self.pause_pub.publish("PAUSE")
                    
            elif self.is_paused:  # Handle pause menu controls
                self.handle_pause_menu(key.lower())
                
            elif self.current_phase == "phase1":  # Handle difficulty selection
                self.handle_difficulty_selection(key)
                
            elif key == 'q' or key == 'Q':  # Quit game
                rospy.signal_shutdown("User quit")
                break
                
            rospy.sleep(0.02)  # Small delay to prevent CPU overuse

if __name__ == '__main__':
    try:
        control_node = ControlNode()
        control_node.run()
    except rospy.ROSInterruptException:
        pass