import pygame
import random

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Catch the Ball")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Player paddle
player_width = 100
player_height = 20
player_x = width // 2 - player_width // 2
player_y = height - 40
player_speed = 5

# Ball
ball_size = 20
ball_x = random.randint(0, width - ball_size)
ball_y = 0
ball_speed = 3

# Game loop
clock = pygame.time.Clock()
score = 0
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Move the player
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT] and player_x > 0:
        player_x -= player_speed
    if keys[pygame.K_RIGHT] and player_x < width - player_width:
        player_x += player_speed

    # Move the ball
    ball_y += ball_speed

    # Check for collision
    if (player_x < ball_x < player_x + player_width and
        player_y < ball_y + ball_size < player_y + player_height):
        score += 1
        ball_x = random.randint(0, width - ball_size)
        ball_y = 0
        ball_speed += 0.5

    # Check if ball is out of bounds
    if ball_y > height:
        ball_x = random.randint(0, width - ball_size)
        ball_y = 0

    # Clear the screen
    screen.fill(BLACK)

    # Draw the player
    pygame.draw.rect(screen, WHITE, (player_x, player_y, player_width, player_height))

    # Draw the ball
    pygame.draw.circle(screen, RED, (ball_x, ball_y), ball_size // 2)

    # Display the score
    font = pygame.font.Font(None, 36)
    text = font.render(f"Score: {score}", True, WHITE)
    screen.blit(text, (10, 10))

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)

pygame.quit()
