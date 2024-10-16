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
dark_green = (0, 100, 0)
dark_blue = (0, 0, 139)
black = (0, 0, 0)
red = (213, 50, 80)
yellow = (255, 255, 102)
white = (255, 255, 255)
obstacle_color = (0, 191, 255)  # Cyan for obstacles

# Window dimensions (full screen)
width, height = pygame.display.Info().current_w, pygame.display.Info().current_h

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

def draw_snake(snake_block, snake_list):
    for segment in snake_list:
        try:
            pygame.draw.rect(window, purple, [segment[0], segment[1], snake_block, snake_block])
        except pygame.error as e:
            print(f"Failed to draw snake segment: {e}")

def draw_obstacle(shape, pos):
    # Draw a simple obstacle with Tetris-like shapes
    try:
        if shape == 'L':
            pygame.draw.rect(window, obstacle_color, [pos[0], pos[1], snake_block, snake_block])
            pygame.draw.rect(window, obstacle_color, [pos[0], pos[1] + snake_block, snake_block, snake_block])
            pygame.draw.rect(window, obstacle_color, [pos[0], pos[1] + 2 * snake_block, snake_block, snake_block])
            pygame.draw.rect(window, obstacle_color, [pos[0] + snake_block, pos[1] + 2 * snake_block, snake_block, snake_block])
        elif shape == 'O':
            pygame.draw.rect(window, obstacle_color, [pos[0], pos[1], snake_block, snake_block])
            pygame.draw.rect(window, obstacle_color, [pos[0] + snake_block, pos[1], snake_block, snake_block])
            pygame.draw.rect(window, obstacle_color, [pos[0], pos[1] + snake_block, snake_block, snake_block])
            pygame.draw.rect(window, obstacle_color, [pos[0] + snake_block, pos[1] + snake_block, snake_block, snake_block])
    except pygame.error as e:
        print(f"Failed to draw obstacle: {e}")

def game_over():
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
        try:
            # Create a semi-transparent surface for the pause panel
            pause_surface = pygame.Surface((width, height))
            pause_surface.set_alpha(128)  # Set alpha for transparency
            pause_surface.fill((255, 255, 255))  # Fill with white for a see-through background
            window.blit(pause_surface, (0, 0))  # Blit the surface onto the window
            # Adjusted font size for better fit
            pause_font = pygame.font.SysFont("timesnewroman", 40)
            pause_message = pause_font.render("Paused. Press P to unpause, R to restart, H for Homepage, or E to exit", True, yellow)
            window.blit(pause_message, [width / 6, height / 3])
            pygame.display.update()
        except pygame.error as e:
            print(f"Failed to display pause screen: {e}")

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    paused = False
                if event.key == pygame.K_r:
                    game_loop()
                if event.key == pygame.K_h:
                    game_intro()
                if event.key == pygame.K_e:
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
    global current_score, treats_eaten, obstacle_active, obstacle_shape, obstacle_position, dark_green
    game_over_flag = False

    x = width / 2
    y = height / 2
    x_change = 0
    y_change = 0
    snake_list = []
    snake_length = 1

    food_x = round(random.randrange(0, width - snake_block) / 40.0) * 40.0
    food_y = round(random.randrange(0, height - snake_block) / 40.0) * 40.0

    while not game_over_flag:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT and x_change != snake_block:
                    x_change = -snake_block
                    y_change = 0
                elif event.key == pygame.K_RIGHT and x_change != -snake_block:
                    x_change = snake_block
                    y_change = 0
                elif event.key == pygame.K_UP and y_change != snake_block:
                    y_change = -snake_block
                    x_change = 0
                elif event.key == pygame.K_DOWN and y_change != -snake_block:
                    y_change = snake_block
                    x_change = 0
                elif event.key == pygame.K_p:
                    game_pause()
                elif event.key == pygame.K_e:
                    pygame.quit()
                    quit()

        if x >= width - snake_block or x < 0 or y >= height - snake_block or y < 0:
            game_over_flag = True

        x += x_change
        y += y_change
        try:
            window.fill(dark_green)
            pygame.draw.rect(window, yellow, [food_x, food_y, snake_block, snake_block])
            # Draw a strong exterior rim to delimit the game area
            pygame.draw.rect(window, black, [0, 0, width, height], 1)
            # Draw a dark blue area outside the gaming map
            pygame.draw.rect(window, dark_blue, [0, 0, width, 40])
            pygame.draw.rect(window, dark_blue, [0, height - 40, width, 40])
            pygame.draw.rect(window, dark_blue, [0, 0, 40, height])
            pygame.draw.rect(window, dark_blue, [width - 40, 0, 40, height])
        except pygame.error as e:
            print(f"Failed to draw food or game rim: {e}")

        snake_head = []
        snake_head.append(x)
        snake_head.append(y)
        snake_list.append(snake_head)

        if len(snake_list) > snake_length:
            del snake_list[0]

        for segment in snake_list[:-1]:
            if segment == snake_head:
                game_over_flag = True

        # Check if the snake has collided with an obstacle
        if obstacle_active:
            if (x, y) in [(obstacle_position[0], obstacle_position[1]), (obstacle_position[0], obstacle_position[1] + snake_block), 
                          (obstacle_position[0], obstacle_position[1] + 2 * snake_block), (obstacle_position[0] + snake_block, obstacle_position[1] + 2 * snake_block)]:
                game_over_flag = True
                window.fill(white)  # Flash white when the snake hits an obstacle

        draw_snake(snake_block, snake_list)

        # Display current score at all times
        try:
            display_message(f"\nSCORE: {current_score}", white, 10)
        except pygame.error as e:
            print(f"Failed to display score: {e}")

        # Draw obstacle if active
        if obstacle_active:
            draw_obstacle(obstacle_shape, obstacle_position)

        pygame.display.update()

        # Check if the snake has eaten the food
        if x == food_x and y == food_y:
            food_x = round(random.randrange(0, width - snake_block) / 40.0) * 40.0
            food_y = round(random.randrange(0, height - snake_block) / 40.0) * 40.0
            snake_length += 1
            current_score += 10
            treats_eaten += 1

            # Obstacle logic: spawn an obstacle after every 3rd treat
            if treats_eaten % 1 == 0:
                obstacle_active = True
                obstacle_shape = random.choice(['L', 'O'])  # Choose a random shape
                obstacle_position = [round(random.randrange(0, width - snake_block) / 40.0) * 40.0,
                                     round(random.randrange(0, height - snake_block) / 40.0) * 40.0]
            else:
                obstacle_active = False  # Obstacle disappears after another treat

            # Increase obstacles and treats every 30 points
            if current_score % 30 == 0:
                obstacle_active = True
                obstacle_shape = random.choice(['L', 'O'])  # Choose a random shape
                obstacle_position = [round(random.randrange(0, width - snake_block) / 40.0) * 40.0,
                                     round(random.randrange(0, height - snake_block) / 40.0) * 40.0]

            # Change background color each time a treat is eaten
            if current_score % 10 == 0:
                dark_green = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

        clock.tick(snake_speed)

    game_over()

game_intro()
