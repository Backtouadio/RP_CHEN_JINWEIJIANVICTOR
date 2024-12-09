Launch all necessary nodes before game_node 

# without pygame
roscore
rosrun RP_CHEN_JINWEIJIANVICTOR result_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR control_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR info_user.py
rosrun RP_CHEN_JINWEIJIANVICTOR game_node.py

# with pygame
roscore
rosrun RP_CHEN_JINWEIJIANVICTOR result_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR control_py_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR info_user.py
rosrun RP_CHEN_JINWEIJIAN game_node.py

Make sure you have pygame installed, and if necessary compile your package using catkin_make or source the bash with source/devel/setup.bash
In my case was not neccesary.
The subscribers and publishers were created properly with three topics, user_information, keyboard_control and result_information.
The proposed nodes were created correctly, info_user publishes to the topic user_information, control_node and control_py_node publishes to keyboard control, and result_information is subscribed to game_node. the game_node is subscribed to the first two topics and publishes the result.
The game phases can be seen in the game_node terminal, and all results or user information received texts can be seen in their respective terminals.
The game plays correctly however user information needs to rerun each time you wanna run game.py, and some other implementations were ignored due to being a little bit hard, like restart, homepage, or pause. I couldnt implement it correctly in the control node so i decided to make it simpler.
We needed to change the location of the user_msg and we load it inside the RP_CHEN_JINWEIJIANVICTOR.