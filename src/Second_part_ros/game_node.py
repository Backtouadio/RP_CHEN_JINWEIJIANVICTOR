#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pygame
import time
import random
import os
import rospy
import math
from std_msgs.msg import String, Int64, Bool
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg
from RP_CHEN_JINWEIJIANVICTOR.srv import GetUserScore, GetUserScoreResponse
from RP_CHEN_JINWEIJIANVICTOR.srv import SetGameDifficulty, SetGameDifficultyResponse

class GameNode:
    def __init__(self):
        """Initialize the game node with ROS and Pygame"""
        # Initialize ROS node
        rospy.init_node('game_node')
        
        # Initialize ROS parameters with defaults
        self.user_name = rospy.get_param('~user_name', 'default_user')
        self.player_color = rospy.get_param('~change_player_color', 1)  # 1:Red, 2:Purple, 3:Blue
        self.screen_param = rospy.get_param('~screen_param', 'phase1')
        
        # Set current phase from parameter
        self.current_phase = self.screen_param
        
        # Game state flags
        self.paused = False
        self.game_over = False
        self.shadow_active = False
        self.color_selected = False  # New flag for color selection
        self.difficulty_selected = False  # New flag for difficulty
        
        # Store all user scores
        self.user_scores = {}
        
        # Obstacle properties
        self.obstacles = []  # List to store active obstacles
        self.shadow_obstacles = []  # List to store shadow previews
        self.min_obstacle_size = 40
        self.max_obstacle_size = 100
        
        # ROS Subscribers
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.control_sub = rospy.Subscriber('keyboard_control', String, self.control_callback)
        
        # ROS Publishers
        self.score_pub = rospy.Publisher('result_information', Int64, queue_size=10)
        self.phase_pub = rospy.Publisher('game_phase', String, queue_size=10)
        
        # ROS Services
        self.score_service = rospy.Service('user_score', GetUserScore, self.handle_get_score)
        self.difficulty_service = rospy.Service('difficulty', SetGameDifficulty, self.handle_set_difficulty)
        
        # Initialize Pygame and game variables
        self.initialize_game()
        
        rospy.loginfo("Game node has started")

    def initialize_game(self):
        """Initialize pygame and game variables"""
        pygame.init()
        
        # Define colors based on player_color parameter
        self.colors = {
            1: (213, 50, 80),    # Red
            2: (160, 32, 240),   # Purple
            3: (0, 0, 139)       # Blue
        }
        self.color_names = {
            1: "Red",
            2: "Purple",
            3: "Blue"
        }
        
        # Basic colors
        self.player_color_rgb = self.colors.get(self.player_color, self.colors[1])
        self.dark_green = (0, 100, 0)
        self.black = (0, 0, 0)
        self.yellow = (255, 255, 102)
        self.white = (255, 255, 255)
        self.obstacle_color = (0, 191, 255)
        self.shadow_color = (128, 128, 128, 128)
        
        # Window setup
        self.width = pygame.display.Info().current_w
        self.height = pygame.display.Info().current_h
        self.playable_area_margin = 60
        self.window = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        pygame.display.set_caption('Snake Game')
        
        # Game variables
        self.clock = pygame.time.Clock()
        self.font_style = pygame.font.SysFont("timesnewroman", 50)
        self.score_font = pygame.font.SysFont("timesnewroman", 70)
        
        # Snake properties
        self.snake_block = 20
        self.snake_speed = 7  # Default speed, will be changed by difficulty
        self.snake_list = []
        self.snake_length = 1
        self.direction = None
        
        # Score tracking
        self.current_score = 0
        self.treats_eaten = 0
        
        # Initial positions
        self.x = self.width / 2
        self.y = self.height / 2
        self.x_change = 0
        self.y_change = 0
        
        # Generate initial food position
        self.generate_food()

    # [Previous methods remain unchanged: generate_food, generate_obstacle_preview, 
    # add_obstacle, draw_obstacle, check_collision]
    def reset_game(self):
        """Reset game state to initial values"""
        self.snake_list = []
        self.snake_length = 1
        self.direction = None
        self.x = self.width / 2
        self.y = self.height / 2
        self.x_change = 0
        self.y_change = 0
        self.current_score = 0
        self.treats_eaten = 0
        self.obstacles = []
        self.shadow_obstacles = []
        self.shadow_active = False
        self.game_over = False
        self.generate_food()  

    def generate_food(self):
        """Generate new food position avoiding obstacles"""
        valid_position = False
        while not valid_position:
            self.food_x = round(random.randrange(
                self.playable_area_margin, 
                self.width - self.snake_block - self.playable_area_margin
            ) / 40.0) * 40.0
            self.food_y = round(random.randrange(
                self.playable_area_margin, 
                self.height - self.snake_block - self.playable_area_margin
            ) / 40.0) * 40.0
            
            # Check if position conflicts with obstacles
            valid_position = True
            for obstacle in self.obstacles:
                if self.check_collision(
                    self.food_x, self.food_y, 
                    obstacle['x'], obstacle['y'],
                    self.snake_block, obstacle['size']
                ):
                    valid_position = False
                    break
            for segment in self.snake_list:
                if segment[0] == self.food_x and segment[1] == self.food_y:
                    valid_position = False
                    break

    def generate_obstacle_preview(self):
        """Generate shadow preview of next obstacle"""
        if not self.shadow_active:
            # More controlled size range
            size_width = random.randint(80, 240)  # Random width between 20-60
            size_height = random.randint(80, 240)  # Random height between 20-60
            
            x = round(random.randrange(
                self.playable_area_margin, 
                self.width - size_width - self.playable_area_margin
            ) / 40.0) * 40.0
            y = round(random.randrange(
                self.playable_area_margin, 
                self.height - size_height - self.playable_area_margin
            ) / 40.0) * 40.0
            
            self.shadow_obstacles = [{
                'x': x,
                'y': y,
                'width': size_width,
                'height': size_height
            }]
            self.shadow_active = True

    def add_obstacle(self):
        """Convert shadow preview to actual obstacle"""
        if self.shadow_active and self.shadow_obstacles:
            self.obstacles.extend(self.shadow_obstacles)
            self.shadow_obstacles = []
            self.shadow_active = False

    def draw_obstacle(self, obstacle, is_shadow=False):
        """Draw a rectangular obstacle or its shadow preview"""
        color = self.shadow_color if is_shadow else self.obstacle_color
        pygame.draw.rect(self.window, color, [
            obstacle['x'], 
            obstacle['y'], 
            obstacle['width'], 
            obstacle['height']
        ])

    def check_collision(self, x1, y1, x2, y2, size1, width2, height2):
        """Check if two rectangles overlap"""
        return (x1 < x2 + width2 and
                x1 + size1 > x2 and
                y1 < y2 + height2 and
                y1 + size1 > y2)

    def display_message(self, msg, color, pos):
        """Display message on screen"""
        message = self.font_style.render(msg, True, color)
        text_rect = message.get_rect(center=(self.width/2, pos))
        self.window.blit(message, text_rect)

        # *************************
    def display_phase(self):
        """Display current game phase"""
        phase_text = ""
        if self.current_phase == "phase1":
            phase_text = "Set yourself up"
        elif self.current_phase == "phase2":
            phase_text = "Phase 2 Gaming"
        elif self.current_phase == "phase3":
            phase_text = "Game Over"
            
        phase_message = self.font_style.render(phase_text, True, self.white)
        self.window.blit(phase_message, (10, 10))
    def handle_get_score(self, req):
        try:
            score = self.user_scores.get(req.username, 0)
            return GetUserScoreResponse(score)
        except Exception as e:
            rospy.logerr(f"Error handling score request: {e}")
            return GetUserScoreResponse(0)

    def handle_set_difficulty(self, req):
        """Service handler for SetGameDifficulty"""
        if self.current_phase != "phase1":
            rospy.logwarn("Cannot change difficulty - not in phase1")
            return SetGameDifficultyResponse(False)
            
        difficulty = req.change_difficulty.lower()
        if difficulty not in ['easy', 'medium', 'hard']:
            rospy.logwarn(f"Invalid difficulty: {difficulty}")
            return SetGameDifficultyResponse(False)
            
        if difficulty == 'easy':
            self.snake_speed = 7
        elif difficulty == 'medium':
            self.snake_speed = 12
        else:  # hard
            self.snake_speed = 20
            
        self.difficulty_selected = True
        if self.color_selected:  # Only transition if color is also selected
            self.current_phase = "phase2"
            self.reset_game()
            
        rospy.loginfo(f"Difficulty changed to: {difficulty}")
        return SetGameDifficultyResponse(True)

    def handle_phase2_gameplay(self):
        """Handle active gameplay in phase2"""
        if not self.paused:
            # Update snake position
            self.x += self.x_change
            self.y += self.y_change
            
            # Check for collisions
            if (self.x < self.playable_area_margin or 
                self.x >= self.width - self.playable_area_margin or 
                self.y < self.playable_area_margin or 
                self.y >= self.height - self.playable_area_margin):
                self.game_over = True
                
            # Check obstacle collisions
            # In handle_phase2_gameplay, update the collision check:
            for obstacle in self.obstacles:
                if self.check_collision(
                    self.x, self.y,
                    obstacle['x'], obstacle['y'],
                    self.snake_block, 
                    obstacle['width'],  # Use width for collision check
                    obstacle['height']  # Use height for collision check
                ):
                    self.game_over = True
                    break
            
            if self.game_over:
                self.current_phase = "phase3"
                return
            
            # Draw game elements
            self.window.fill(self.dark_green)
            
            # Draw boundaries
            pygame.draw.rect(self.window, self.black, [0, 0, self.width, self.playable_area_margin])
            pygame.draw.rect(self.window, self.black, [0, self.height - self.playable_area_margin, self.width, self.playable_area_margin])
            pygame.draw.rect(self.window, self.black, [0, 0, self.playable_area_margin, self.height])
            pygame.draw.rect(self.window, self.black, [self.width - self.playable_area_margin, 0, self.playable_area_margin, self.height])
            
            # Draw food
            pygame.draw.rect(self.window, self.yellow, [self.food_x, self.food_y, self.snake_block, self.snake_block])
            
            # Draw obstacles
            for obstacle in self.obstacles:
                self.draw_obstacle(obstacle)
            
            # Draw shadow preview if active
            if self.shadow_active:
                for shadow in self.shadow_obstacles:
                    self.draw_obstacle(shadow, is_shadow=True)
            
            # Update and draw snake
            snake_head = [self.x, self.y]
            self.snake_list.append(snake_head)
            if len(self.snake_list) > self.snake_length:
                del self.snake_list[0]
            
            # Check for self-collision
            for block in self.snake_list[:-1]:
                if block == snake_head:
                    self.game_over = True
                    self.current_phase = "phase3"
                    break
            
            # Draw snake with player's color
            for block in self.snake_list:
                pygame.draw.rect(self.window, self.player_color_rgb, 
                            [block[0], block[1], self.snake_block, self.snake_block])
            
            # Check for food collision
            if self.x == self.food_x and self.y == self.food_y:
                self.food_x = round(random.randrange(self.playable_area_margin, 
                                self.width - self.snake_block - self.playable_area_margin) / 40.0) * 40.0
                self.food_y = round(random.randrange(self.playable_area_margin, 
                                self.height - self.snake_block - self.playable_area_margin) / 40.0) * 40.0
                self.snake_length += 1
                self.current_score += 10
                self.treats_eaten += 1
                
                # Add obstacle after eating
                self.add_obstacle()
                # Generate new shadow preview
                self.generate_obstacle_preview()
            
            # Display score and phase
            self.display_message(f"Score: {self.current_score}", self.white, 20)
            self.display_phase()
    

    def handle_keyboard_events(self):
        """Handle keyboard events for game controls"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.cleanup_and_exit()
                return True
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p and self.current_phase == "phase2":
                    self.paused = not self.paused
                    
                elif self.current_phase == "phase1" and not self.color_selected:
                    # Color selection in phase1
                    if event.key in [pygame.K_1, pygame.K_2, pygame.K_3]:
                        color = event.key - pygame.K_1 + 1  # Convert to 1, 2, or 3
                        self.player_color = color
                        self.player_color_rgb = self.colors[color]
                        rospy.set_param('~change_player_color', color)
                        self.color_selected = True
                        rospy.loginfo(f"Color selected: {self.color_names[color]}")
                # Add this new elif block to handle phase3 (game over) controls
                elif self.current_phase == "phase3":
                    if event.key == pygame.K_r:  # Restart
                        self.current_phase = "phase2"
                        self.game_over = False
                        self.reset_game()
                    elif event.key == pygame.K_h:  # Home
                        self.current_phase = "phase1"
                        self.color_selected = False
                        self.difficulty_selected = False
                        self.game_over = False
                        self.reset_game()
                    elif event.key == pygame.K_e:  # Exit
                        self.cleanup_and_exit()
                        return True                       
                elif self.paused:
                    # Pause menu controls
                    if event.key == pygame.K_h:  # Home
                        self.current_phase = "phase1"
                        self.color_selected = False
                        self.difficulty_selected = False
                        self.reset_game()
                        self.paused = False
                    elif event.key == pygame.K_r:  # Restart
                        self.reset_game()
                        self.paused = False
                    elif event.key == pygame.K_e:  # Exit
                        self.cleanup_and_exit()
                        return True
        return False

    def user_callback(self, data):
        """Handle received user information"""
        if self.current_phase == "phase1":
            self.player_name = data.name
            self.player_username = data.username
            self.player_age = data.age
            rospy.loginfo(f"Welcome {self.player_username} of age {self.player_age}!")

    def control_callback(self, data):
        """Handle movement commands from control node"""
        if not self.paused and self.current_phase == "phase2":
            command = data.data
            if command == "UP" and self.direction != "DOWN":
                self.y_change = -self.snake_block
                self.x_change = 0
                self.direction = "UP"
            elif command == "DOWN" and self.direction != "UP":
                self.y_change = self.snake_block
                self.x_change = 0
                self.direction = "DOWN"
            elif command == "LEFT" and self.direction != "RIGHT":
                self.x_change = -self.snake_block
                self.y_change = 0
                self.direction = "LEFT"
            elif command == "RIGHT" and self.direction != "LEFT":
                self.x_change = self.snake_block
                self.y_change = 0
                self.direction = "RIGHT"

    def draw_phase1_screen(self):
        """Draw the phase1 (setup) screen"""
        self.window.fill(self.dark_green)
        if hasattr(self, 'player_username'):
            if self.player_age >= 18:
                self.display_message(f"Welcome {self.player_username} of age {self.player_age} ;) <3)!", self.yellow, self.height/4)
            else:
            
                self.display_message(f"Welcome {self.player_username} of age {self.player_age} :()!", self.yellow, self.height/4)
            if not self.color_selected:
                self.display_message("Select your snakes favorite color!!", self.white, self.height/2 - 60)
                self.display_message("1 - Red", self.colors[1], self.height/2)
                self.display_message("2 - Purple", self.colors[2], self.height/2 + 60)
                self.display_message("3 - Blue", self.colors[3], self.height/2 + 120)
            elif not self.difficulty_selected:
                self.display_message("Waiting for difficulty selection...", self.white, self.height/2)
                self.display_message("(Please use service call to set difficulty)", self.white, self.height/2 + 60)
        else:
            self.display_message("Waiting for your age and stuff...", self.white, self.height/2)

    def draw_pause_menu(self):
        """Draw pause menu overlay"""
        # Semi-transparent overlay
        overlay = pygame.Surface((self.width, self.height))
        overlay.set_alpha(128)
        overlay.fill(self.black)
        self.window.blit(overlay, (0,0))
        
        # Menu options
        self.display_message("GAME PAUSED", self.white, self.height/2 - 120)
        self.display_message("H - Return to Home", self.white, self.height/2 - 40)
        self.display_message("R - Restart Game", self.white, self.height/2 + 40)
        self.display_message("E - Exit/quit Game", self.white, self.height/2 + 120)

    def run(self):
        """Main game loop"""
        while not rospy.is_shutdown():
            # Handle keyboard events
            if self.handle_keyboard_events():
                return

            # Phase-specific updates
            if self.current_phase == "phase1":
                self.draw_phase1_screen()
                
            elif self.current_phase == "phase2":
                if not self.paused:
                    self.handle_phase2_gameplay()
                else:
                    self.draw_pause_menu()
                    
            elif self.current_phase == "phase3":
                # Game over screen
                self.window.fill(self.dark_green)
                self.display_message(f"Game Over! Final Score: {self.current_score}", self.yellow, self.height/2)
                self.display_message("Press R to Restart, E to Exit, H for homepage", self.white, self.height/2 + 100)
                
                # Publish final score
                self.score_pub.publish(self.current_score)
                self.user_scores[self.player_username] = self.current_score
            
            # Update display and maintain frame rate
            pygame.display.update()
            self.clock.tick(self.snake_speed)
            
            # Publish current phase
            self.phase_pub.publish(self.current_phase)

if __name__ == '__main__':
    try:
        game_node = GameNode()
        game_node.run()
    except rospy.ROSInterruptException:
        pass