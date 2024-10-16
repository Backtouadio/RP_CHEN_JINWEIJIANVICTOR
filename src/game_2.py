import pygame
import time
import random
import os

# Initialize the game
try:
    pygame.init()
except pygame.error as e:
    print(f"Failed to initialize pygame: {e}")
    pygame.quit()
    quit()

# Colors
purple = (160, 32, 240)
global dark_green
dark_green = (0, 100, 0)
dark_blue = (0, 0, 139)  # Dark blue for the outside terrain
black = (0, 0, 0)
red = (213, 50, 80)
yellow = (255, 255, 102)
white = (255, 255, 255)
obstacle_color = (0, 191, 255)  # Cyan for obstacles

# Window dimensions (full screen)
width, height = pygame.display.Info().current_w, pygame.display.Info().current_h
playable_area_margin = 60  # Margin for the non-playable visual rim

# Create the game window
try:
    window = pygame.display.set_mode((width, height), pygame.FULLSCREEN)
    pygame.display.set_caption('Snake Game')
except pygame.error as e:
    print(f"Failed to set display mode: {e}")
    pygame.quit()
    quit()

# Clock and font
clock = pygame.time.Clock()
font_style = pygame.font.SysFont("timesnewroman", 50)
score_font = pygame.font.SysFont("timesnewroman", 70)

# Snake properties (smaller size)
snake_block = 20
snake_speed_easy = 7  # Reduced speed for easy mode
snake_speed_medium = 12  # Reduced speed for medium mode
snake_speed_hard = 17  # Reduced speed for hard mode
snake_speed = snake_speed_easy

# Score tracking
current_score = 0
high_score = 0
treats_eaten = 0  # Track how many treats have been eaten for obstacle logic

# Load high score from file if it exists
if os.path.exists('high_score.txt'):
    with open('high_score.txt', 'r') as file:
        high_score = int(file.read())

# Obstacles
obstacle_active = False
obstacle_shape = None
obstacle_position = None

# Pause functionality
pause = False

def display_message(msg, color, pos):
    try:
        message = font_style.render(msg.upper(), True, color)
        text_rect = message.get_rect(center=(width/2, pos))
        window.blit(message, text_rect)
    except pygame.error as e:
        print(f"Failed to display message: {e}")
# Movement direction
direction = None  # Track the current direction

def game_over():
    global dark_green 
    game_over = True
    global high_score, current_score, treats_eaten
    if current_score > high_score:
        high_score = current_score
        # Save high score to file
        with open('high_score.txt', 'w') as file:
            file.write(str(high_score))
    while game_over:
        try:
            window.fill(dark_green)
            display_message(f"Game Over! Your Score: {current_score} | High Score: {high_score}", yellow, height/2)
            display_message("Press R to restart, H for Homepage, or E to exit", white, height/2 + 100)
            pygame.display.update()
        except pygame.error as e:
            print(f"Failed to display game over screen: {e}")
        
        treats_eaten = 0  # Reset treats counter

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    game_loop()
                if event.key == pygame.K_h:
                    game_intro()
                if event.key == pygame.K_e:
                    pygame.quit()
                    quit()
    


def game_pause():
    paused = True
    while paused:
        # Create a semi-transparent surface for the pause panel
        pause_surface = pygame.Surface((width, height))
        pause_surface.set_alpha(0)  # Set alpha for transparency (0-255)
        pause_surface.fill((0, 0, 0))  # Fill with black color for a dark overlay
        window.blit(pause_surface, (0, 0))  # Blit the surface onto the window

        # Adjusted font size for better fit
        pause_font = pygame.font.SysFont("timesnewroman", 40)
        pause_message = pause_font.render("Paused\n Press P to unpause, R to restart, H for Homepage, or E to exit", True, yellow)
        
        # Draw the pause message centered
        message_rect = pause_message.get_rect(center=(width / 2, height / 2))
        window.blit(pause_message, message_rect)

        pygame.display.update()  # Update the display

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:  # Unpause
                    paused = False
                if event.key == pygame.K_r:  # Restart the game
                    game_loop()
                if event.key == pygame.K_h:  # Go to homepage
                    game_intro()
                if event.key == pygame.K_e:  # Exit the game
                    pygame.quit()
                    quit()


def game_intro():
    intro = True
    global snake_speed
    global dark_green
    

    while intro:
        try:
            window.fill(dark_green)
            # Adjusted font size for better fit
            intro_font = pygame.font.SysFont("timesnewroman", 30)
            intro_message1 = intro_font.render("Welcome to Snake Game!", True, red)
            intro_message2 = intro_font.render("Choose your speed: 1-Easy | 2-Medium | 3-Hard!!!", True, white)
            intro_message3 = intro_font.render(f"Highest Score: {high_score}", True, white)
            intro_message4 = intro_font.render("Controls: Arrow keys to move | P to pause | E to exit", True, white)
            window.blit(intro_message1, [width / 6, height / 6])
            window.blit(intro_message2, [width / 6, height / 4])
            window.blit(intro_message3, [width / 4, height / 3])
            window.blit(intro_message4, [width / 6, height / 2])
            pygame.display.update()
        except pygame.error as e:
            print(f"Failed to display intro screen: {e}")

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    snake_speed = snake_speed_easy
                    game_loop()
                if event.key == pygame.K_2:
                    snake_speed = snake_speed_medium
                    game_loop()
                if event.key == pygame.K_3:
                    snake_speed = snake_speed_hard
                    game_loop()
                if event.key == pygame.K_e:
                    pygame.quit()
                    quit()
def game_loop():
    global current_score, treats_eaten, obstacle_active, obstacle_shape, obstacle_position, direction, dark_green
    current_score = 0
    game_over_flag = False

    # Initial snake position at the center of the playable area
    x = width / 2
    y = height / 2
    x_change = 0
    y_change = 0
    snake_list = []
    snake_length = 1

    # Place the treat within the playable area (not outside the margin)
    food_x = round(random.randrange(playable_area_margin, width - snake_block - playable_area_margin) / 40.0) * 40.0
    food_y = round(random.randrange(playable_area_margin, height - snake_block - playable_area_margin) / 40.0) * 40.0

    while not game_over_flag:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT and direction != "RIGHT":
                    x_change = -snake_block
                    y_change = 0
                    direction = "LEFT"
                elif event.key == pygame.K_RIGHT and direction != "LEFT":
                    x_change = snake_block
                    y_change = 0
                    direction = "RIGHT"
                elif event.key == pygame.K_UP and direction != "DOWN":
                    y_change = -snake_block
                    x_change = 0
                    direction = "UP"
                elif event.key == pygame.K_DOWN and direction != "UP":
                    y_change = snake_block
                    x_change = 0
                    direction = "DOWN"
                elif event.key == pygame.K_p:
                    game_pause()
                elif event.key == pygame.K_e:
                    pygame.quit()
                    quit()

        # Update snake's position
        x += x_change
        y += y_change

        # Check for wall collision (ensure snake stays within the playable area)
        if x < playable_area_margin or x >= width - playable_area_margin or y < playable_area_margin or y >= height - playable_area_margin:
            game_over_flag = True  # Trigger game over if the snake hits the boundary

        # Fill terrain outside the rim with dark blue
        window.fill(dark_green)
        pygame.draw.rect(window, black, [0, 0, width, 40])
        pygame.draw.rect(window, black, [0, height - 40, width, 40])
        pygame.draw.rect(window, black, [0, 0, 40, height])
        pygame.draw.rect(window, black, [width - 40, 0, 40, height])

        # Draw the playable area boundary (rim)
        #pygame.draw.rect(window, dark_blue, [playable_area_margin, playable_area_margin, width - 2 * playable_area_margin, height - 2 * playable_area_margin], 5)
        # Draw food (treat) inside the playable area
        pygame.draw.rect(window, yellow, [food_x, food_y, snake_block, snake_block])

        # Draw the snake
        snake_head = [x, y]
        snake_list.append(snake_head)
        if len(snake_list) > snake_length:
            del snake_list[0]

        for block in snake_list[:-1]:
            if block == snake_head:
                game_over_flag = True

        for block in snake_list:
            pygame.draw.rect(window, purple, [block[0], block[1], snake_block, snake_block])

        # Check if the snake eats the food
        if x == food_x and y == food_y:
            food_x = round(random.randrange(playable_area_margin, width - snake_block - playable_area_margin) / 40.0) * 40.0
            food_y = round(random.randrange(playable_area_margin, height - snake_block - playable_area_margin) / 40.0) * 40.0
            snake_length += 1
            treats_eaten += 1
            current_score += 10

            # Every third treat, spawn an obstacle
            if treats_eaten % 1 == 0:
                obstacle_active = True
                # Ensure the obstacle is within the playable area
                obstacle_x = round(random.randrange(playable_area_margin, width - snake_block - playable_area_margin) / 40.0) * 40.0
                obstacle_y = round(random.randrange(playable_area_margin, height - snake_block - playable_area_margin) / 40.0) * 40.0
                obstacle_position = (obstacle_x, obstacle_y)
                # Define the shape of the obstacle (inspired by Tetris blocks)
                obstacle_shape = random.choice([
                    [(0, 0), (0, 20), (20, 20), (20, 0)],  # Square
                    [(0, 0), (20, 0), (40, 0), (60, 0)],  # Line
                    [(0, 0), (20, 0), (20, 20), (40, 20)]  # L-shape
                ])
            if current_score % 10 == 0:
                dark_green = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
         # Display current score at all times
        try:
            display_message(f"SCORE: {current_score}", white, 20)
        except pygame.error as e:
            print(f"Failed to display score: {e}")

        # Draw obstacles if active
        if obstacle_active:
            for block in obstacle_shape:
                pygame.draw.rect(window, obstacle_color, [obstacle_position[0] + block[0], obstacle_position[1] + block[1], snake_block, snake_block])

            # Check if the snake hits the obstacle
            for block in obstacle_shape:
                if (x == obstacle_position[0] + block[0]) and (y == obstacle_position[1] + block[1]):
                    game_over_flag = True

        # Check for collision with the boundary (the "rim")
        if x < playable_area_margin or x >= width - playable_area_margin or y < playable_area_margin or y >= height - playable_area_margin:
            game_over_flag = True

        pygame.display.update()

        clock.tick(snake_speed)

    # End game
    game_over()

game_intro()

