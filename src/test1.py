import pygame
import random

# Initialize pygame
pygame.init()

# Screen settings
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 400
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Snake Game")

# Colors
PURPLE = (128, 0, 128)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Game clock
clock = pygame.time.Clock()
FPS = 15  # Initial frames per second, increases as difficulty rises

# Snake settings
SNAKE_SIZE = 20
snake_pos = [[100, 100]]  # Snake starting position (list of segments)
snake_dir = "RIGHT"
snake_speed = SNAKE_SIZE  # Snake moves by one cube size per tick

# Food settings
food_pos = [random.randrange(1, SCREEN_WIDTH // SNAKE_SIZE) * SNAKE_SIZE,
            random.randrange(1, SCREEN_HEIGHT // SNAKE_SIZE) * SNAKE_SIZE]

# Score
score = 0
font = pygame.font.SysFont(None, 36)

# Game states
game_over = False
game_started = False

# Function to display text on the screen
def draw_text(text, font, color, x, y):
    screen_text = font.render(text, True, color)
    screen.blit(screen_text, (x, y))

# Function to handle food generation
def spawn_food():
    return [random.randrange(1, SCREEN_WIDTH // SNAKE_SIZE) * SNAKE_SIZE,
            random.randrange(1, SCREEN_HEIGHT // SNAKE_SIZE) * SNAKE_SIZE]

# Collision detection for snake eating food or hitting walls or itself
def check_collisions(snake_pos, food_pos):
    if snake_pos[0] == food_pos:  # Snake eats food
        return "FOOD"
    if (snake_pos[0][0] >= SCREEN_WIDTH or snake_pos[0][0] < 0 or
        snake_pos[0][1] >= SCREEN_HEIGHT or snake_pos[0][1] < 0):  # Hits wall
        return "WALL"
    if snake_pos[0] in snake_pos[1:]:  # Hits itself
        return "SELF"
    return None

# Function to update snake's position
def update_snake(snake_pos, snake_dir):
    head_x, head_y = snake_pos[0]

    if snake_dir == "RIGHT":
        new_head = [head_x + snake_speed, head_y]
    elif snake_dir == "LEFT":
        new_head = [head_x - snake_speed, head_y]
    elif snake_dir == "UP":
        new_head = [head_x, head_y - snake_speed]
    elif snake_dir == "DOWN":
        new_head = [head_x, head_y + snake_speed]

    snake_pos = [new_head] + snake_pos[:-1]
    return snake_pos

# Main game loop
def game_loop():
    global snake_pos, snake_dir, score, game_over, game_started, food_pos, FPS

    # Reset game state
    snake_pos = [[100, 100]]
    snake_dir = "RIGHT"
    score = 0
    game_over = False
    food_pos = spawn_food()
    FPS = 15

    while True:
        screen.fill(BLACK)

        # Welcome screen
        if not game_started:
            draw_text("Snake Game", font, WHITE, SCREEN_WIDTH // 2 - 100, SCREEN_HEIGHT // 2 - 50)
            draw_text("Press any key to start", font, WHITE, SCREEN_WIDTH // 2 - 150, SCREEN_HEIGHT // 2)
            pygame.display.update()

            # Wait for key press to start the game
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
                if event.type == pygame.KEYDOWN:
                    game_started = True
            continue

        # Game over screen
        if game_over:
            draw_text("Game Over", font, RED, SCREEN_WIDTH // 2 - 100, SCREEN_HEIGHT // 2 - 50)
            draw_text(f"Final Score: {score}", font, WHITE, SCREEN_WIDTH // 2 - 100, SCREEN_HEIGHT // 2)
            draw_text("Press any key to restart", font, WHITE, SCREEN_WIDTH // 2 - 150, SCREEN_HEIGHT // 2 + 50)
            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
                if event.type == pygame.KEYDOWN:
                    game_loop()  # Restart the game
            continue

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # Player movement
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT] and snake_dir != "RIGHT":
            snake_dir = "LEFT"
        if keys[pygame.K_RIGHT] and snake_dir != "LEFT":
            snake_dir = "RIGHT"
        if keys[pygame.K_UP] and snake_dir != "DOWN":
            snake_dir = "UP"
        if keys[pygame.K_DOWN] and snake_dir != "UP":
            snake_dir = "DOWN"

        # Update snake position
        snake_pos = update_snake(snake_pos, snake_dir)

        # Check for collisions
        collision_result = check_collisions(snake_pos, food_pos)
        if collision_result == "FOOD":
            # Snake grows when it eats food
            snake_pos.append(snake_pos[-1])
            score += 10
            food_pos = spawn_food()
            FPS += 1  # Increase difficulty as score increases
        elif collision_result in ["WALL", "SELF"]:
            game_over = True

        # Drawing snake and food
        for segment in snake_pos:
            pygame.draw.rect(screen, PURPLE, (segment[0], segment[1], SNAKE_SIZE, SNAKE_SIZE))
        pygame.draw.rect(screen, GREEN, (food_pos[0], food_pos[1], SNAKE_SIZE, SNAKE_SIZE))

        # Display the score
        draw_text(f"Score: {score}", font, WHITE, 10, 10)

        # Update display
        pygame.display.update()

        # Frame rate
        clock.tick(FPS)

# Start the game loop
game_loop()
