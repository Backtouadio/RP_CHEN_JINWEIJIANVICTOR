#!/usr/bin/env python3

import pygame
import time
import random
import os
import rospy
from std_msgs.msg import String, Int64
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg
from RP_CHEN_JINWEIJIANVICTOR.srv import GetUserScore, GetUserScoreResponse
from RP_CHEN_JINWEIJIANVICTOR.srv import SetGameDifficulty, SetGameDifficultyResponse

class GameNode:
    def __init__(self):
        """Initialize the game node with ROS and Pygame"""
        # Initialize ROS node
        rospy.init_node('game_node')
        
        # Initialize parameters
        self.user_name = rospy.get_param('~user_name', 'default_user')
        self.player_color = rospy.get_param('~change_player_color', 1)  # 1:Red, 2:Purple, 3:Blue
        self.screen_param = rospy.get_param('~screen_param', 'phase1')
        
        # Store all user scores
        self.user_scores = {}
        
        # ROS Subscribers and Publishers
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.control_sub = rospy.Subscriber('keyboard_control', String, self.control_callback)
        self.score_pub = rospy.Publisher('result_information', Int64, queue_size=10)
        
        # ROS Services
        self.score_service = rospy.Service('user_score', GetUserScore, self.handle_get_score)
        self.difficulty_service = rospy.Service('difficulty', SetGameDifficulty, self.handle_set_difficulty)
        
        # Game phase tracking
        self.current_phase = self.screen_param
        
        # Initialize Pygame and game variables
        self.initialize_game()
        
        rospy.loginfo("Game node has started")

    def handle_get_score(self, req):
        """Service handler for GetUserScore"""
        score = self.user_scores.get(req.username, 0)
        return GetUserScoreResponse(score)
    
    def handle_set_difficulty(self, req):
        """Service handler for SetGameDifficulty"""
        if self.current_phase != "phase1":
            rospy.logwarn("Cannot change difficulty - not in phase1")
            return SetGameDifficultyResponse(False)
            
        difficulty = req.change_difficulty.lower()
        if difficulty not in ['easy', 'medium', 'hard']:
            rospy.logwarn(f"Invalid difficulty: {difficulty}")
            return SetGameDifficultyResponse(False)
            
        # Set game speed based on difficulty
        if difficulty == 'easy':
            self.snake_speed = 7
        elif difficulty == 'medium':
            self.snake_speed = 12
        else:  # hard
            self.snake_speed = 17
            
        rospy.loginfo(f"Difficulty changed to: {difficulty}")
        return SetGameDifficultyResponse(True)

    def initialize_game(self):
        """Initialize pygame and game variables"""
        try:
            pygame.init()
            
            # Define colors based on player_color parameter
            self.colors = {
                1: (213, 50, 80),    # Red
                2: (160, 32, 240),   # Purple
                3: (0, 0, 139)       # Blue
            }
            
            self.player_color_rgb = self.colors.get(self.player_color, self.colors[1])
            
            # Colors
            self.dark_green = (0, 100, 0)
            self.black = (0, 0, 0)
            self.yellow = (255, 255, 102)
            self.white = (255, 255, 255)
            self.obstacle_color = (0, 191, 255)
            
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
            self.snake_speed = 7  # Default speed, will be changed by user selection
            self.current_score = 0
            self.treats_eaten = 0
            
            # Player information
            self.player_name = None
            self.player_username = None
            self.player_age = None
            
            # Movement
            self.direction = None
            self.x_change = 0
            self.y_change = 0
            
            # Obstacles
            self.obstacle_active = False
            self.obstacle_shape = None
            self.obstacle_position = None
            
        except pygame.error as e:
            rospy.logerr(f"Failed to initialize pygame: {e}")
            pygame.quit()
            quit()
            
    def user_callback(self, data):
        """Handle received user information"""
        if self.current_phase == "phase1":
            self.player_name = data.name
            self.player_username = data.username
            self.player_age = data.age
            rospy.loginfo(f"Welcome {self.player_name}!")
            # Don't start game immediately, let user choose speed first
            self.waiting_for_speed = True
            
    def control_callback(self, data):
        """Handle received control commands"""
        if self.current_phase == "phase2":  # Changed from GAME to phase2
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
                
    def display_message(self, msg, color, pos):
        """Display message on screen"""
        try:
            message = self.font_style.render(msg.upper(), True, color)
            text_rect = message.get_rect(center=(self.width/2, pos))
            self.window.blit(message, text_rect)
        except pygame.error as e:
            rospy.logerr(f"Failed to display message: {e}")
            
    def start_game(self):
        """Initialize and start the game"""
        self.x = self.width / 2
        self.y = self.height / 2
        self.x_change = 0
        self.y_change = 0
        self.snake_list = []
        self.snake_length = 1
        self.current_score = 0
        self.treats_eaten = 0
        
        # Initial food position
        self.food_x = round(random.randrange(self.playable_area_margin, self.width - self.snake_block - self.playable_area_margin) / 40.0) * 40.0
        self.food_y = round(random.randrange(self.playable_area_margin, self.height - self.snake_block - self.playable_area_margin) / 40.0) * 40.0
        
        self.game_loop()
        
    def game_loop(self):
        """Main game loop"""
        game_over_flag = False
        
        # Update phase to phase2 (gameplay)
        self.current_phase = "phase2"
        rospy.set_param('~screen_param', self.current_phase)
        
        while not game_over_flag and not rospy.is_shutdown():
            # Update snake position
            self.x += self.x_change
            self.y += self.y_change
            
            # Check for collisions
            if (self.x < self.playable_area_margin or 
                self.x >= self.width - self.playable_area_margin or 
                self.y < self.playable_area_margin or 
                self.y >= self.height - self.playable_area_margin):
                game_over_flag = True
                
            # Draw game elements
            self.window.fill(self.dark_green)
            
            # Draw boundaries
            pygame.draw.rect(self.window, self.black, [0, 0, self.width, 40])
            pygame.draw.rect(self.window, self.black, [0, self.height - 40, self.width, 40])
            pygame.draw.rect(self.window, self.black, [0, 0, 40, self.height])
            pygame.draw.rect(self.window, self.black, [self.width - 40, 0, 40, self.height])
            
            # Draw food
            pygame.draw.rect(self.window, self.yellow, [self.food_x, self.food_y, self.snake_block, self.snake_block])
            
            # Update and draw snake
            snake_head = [self.x, self.y]
            self.snake_list.append(snake_head)
            if len(self.snake_list) > self.snake_length:
                del self.snake_list[0]
                
            # Check for self-collision
            for block in self.snake_list[:-1]:
                if block == snake_head:
                    game_over_flag = True
                    
            # Draw snake with player's color
            for block in self.snake_list:
                pygame.draw.rect(self.window, self.player_color_rgb, [block[0], block[1], self.snake_block, self.snake_block])
                
            # Check for food collision
            if self.x == self.food_x and self.y == self.food_y:
                self.food_x = round(random.randrange(self.playable_area_margin, self.width - self.snake_block - self.playable_area_margin) / 40.0) * 40.0
                self.food_y = round(random.randrange(self.playable_area_margin, self.height - self.snake_block - self.playable_area_margin) / 40.0) * 40.0
                self.snake_length += 1
                self.current_score += 10
                
            # Display score
            self.display_message(f"Score: {self.current_score}", self.white, 20)
            
            pygame.display.update()
            self.clock.tick(self.snake_speed)
            
        # Game over - publish score and transition to phase3
        self.current_phase = "phase3"
        rospy.set_param('~screen_param', self.current_phase)
        rospy.loginfo("Game Over! Publishing final score")
        self.score_pub.publish(self.current_score)
        
        # Store the score for the service
        self.user_scores[self.user_name] = self.current_score
        
        # Display game over message
        self.window.fill(self.dark_green)
        self.display_message(f"Game Over! Final Score: {self.current_score}", self.yellow, self.height/2)
        self.display_message("Press Q to quit", self.white, self.height/2 + 100)
        pygame.display.update()
        
        # Wait for quit
        waiting = True
        while waiting and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                    waiting = False
                    pygame.quit()
                    
    def run(self):
        """Main node running loop"""
        self.waiting_for_speed = False
        # Display welcome screen while waiting for player info
        while not rospy.is_shutdown():
            if self.current_phase == "phase1":
                self.window.fill(self.dark_green)
                
                if not self.player_name:  # Still waiting for player info
                    self.display_message("Waiting for player information...", self.white, self.height/2)
                else:  # Got player info, show welcome screen with speed selection
                    # Display player information
                    self.display_message(f"Welcome {self.player_name}!", self.yellow, self.height/4)
                    self.display_message(f"Username: {self.player_username}", self.white, self.height/3)
                    self.display_message(f"Age: {self.player_age}", self.white, self.height/2.5)
                    
                    # Display speed selection
                    self.display_message("Choose your speed:", self.white, self.height/2)
                    self.display_message("1 - Easy (Press 1)", self.white, self.height/1.8)
                    self.display_message("2 - Medium (Press 2)", self.white, self.height/1.6)
                    self.display_message("3 - Hard (Press 3)", self.white, self.height/1.4)
                    
                    # Handle speed selection
                    for event in pygame.event.get():
                        if event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_1:
                                self.snake_speed = 7  # Easy speed
                                self.start_game()
                            elif event.key == pygame.K_2:
                                self.snake_speed = 12  # Medium speed
                                self.start_game()
                            elif event.key == pygame.K_3:
                                self.snake_speed = 17  # Hard speed
                                self.start_game()
                            elif event.key == pygame.K_q:  # Option to quit
                                pygame.quit()
                                return
                
                pygame.display.update()
                self.clock.tick(30)
            elif self.current_phase == "phase2":
                break  # Exit this loop when game starts
                
            rospy.sleep(0.1)  # Small delay to prevent CPU overuse
        
        rospy.spin()

if __name__ == '__main__':
    try:
        game_node = GameNode()
        game_node.run()
    except rospy.ROSInterruptException:
        pass