# Snake Game

## Overview
This is a simple implementation of the classic Snake Game using the Pygame library in Python. The player controls a snake that moves around the screen, eating treats to grow longer while avoiding collisions with the walls and itself. The game features different speed settings, a scoring system, and obstacles that appear as the player progresses.

## Features
- **Full-Screen Mode**: The game runs in full-screen mode, utilizing the entire display.
- **Speed Selection**: Players can choose from three difficulty levels: Easy, Medium, and Hard, which affect the snake's speed.
- **Score Tracking**: The game keeps track of the current score and the highest score, which is saved to a file.
- **Obstacles**: Every few treats eaten, obstacles appear on the screen, adding an extra challenge.
- **Pause Functionality**: Players can pause the game and resume it at any time.
- **Game Over Screen**: When the game ends, a game over screen displays the player's score and options to restart or exit.

## Controls
- **Arrow Keys**: Control the direction of the snake (Up, Down, Left, Right).
- **P**: Pause the game.
- **R**: Restart the game after a game over.
- **H**: Go to the homepage (intro screen).
- **E**: Exit the game.

## Requirements
- Python 3.x
- Pygame library (install via `pip install pygame`)

## How to Run
1. Ensure you have Python and Pygame installed.
2. Clone or download the repository.
3. Navigate to the directory containing `game.py`.
4. Run the game using the command:
   ```bash
   python src/game.py
   ```

