A launcher was created in order to facilitate the initialization of all nodes

# without pygame
 roslaunch RP_CHEN_JINWEIJIANVICTOR game_control.launch

# with pygame
 roslaunch RP_CHEN_JINWEIJIANVICTOR game_pygame.launch

Make sure you have pygame installed, and if necessary compile your package using catkin_make or source the bash with source/devel/setup.bash
In my case was not neccesary, my terminal automatically sources when is open.

The subscribers and publishers were created properly with three topics, user_information, keyboard_control and result_information.
The proposed nodes were created correctly, info_user publishes to the topic user_information, control_node and control_py_node publishes to keyboard control, and result_information is subscribed to game_node. the game_node is subscribed to the first two topics and publishes the result.

The game phases can be seen in the game window and additionally in the new terminal created for game control (this was done in the launcher).
Solved some issues in first part ros, where i couldnt implemente the pause button and i had problems with the game over panel.

*new implementations
-Pause button with restart, homepage and exit options.
-Game over panel, with also restart, homepage and exit options.
-Now the player can choose the color of the snake.
-Now when a treat is eaten by the snake, a grey shadow will appear in the screen, this just shows you where the next obstacle is gonna appear, when a treat is eaten while the shadow is showing, the block appears and changes color to blue and it acts as an obstacle (grey areas does not count as obstacles).
-Game difficulty is set via service, this is requested in a ner ubuntu terminal.
-Params where created.
